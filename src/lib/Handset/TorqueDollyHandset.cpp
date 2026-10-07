#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_TX) && defined(TORQUE_DOLLY_HANDSET)

#include "TorqueDollyHandset.h"
#include "crsf_protocol.h"
#include "common.h"

namespace
{
constexpr int PIN_X = 1;
constexpr int PIN_Y = 2;
constexpr int PIN_READY_SWITCH = 44;
constexpr int PIN_STATUS_LED = 43;

constexpr float EMA_ALPHA = 0.20f;
constexpr float DEADBAND = 2.0f;

constexpr float ACCEL_RATE = 24.0f;
constexpr float DECEL_RATE_GENTLE = 30.0f;
constexpr float DECEL_RATE_STOP = 70.0f;

constexpr float FULL_STEERING_AUTHORITY = 70.0f;
constexpr float PIVOT_TURN_MAX = 40.0f;
constexpr float PIVOT_BLEND_END = 40.0f;
constexpr float STEERING_ACCEL_RATE_STANDSTILL = 200.0f;
constexpr float STEERING_ACCEL_RATE_MOVING = 90.0f;
constexpr float STEERING_DECEL_RATE_STANDSTILL = 50.0f;
constexpr float STEERING_DECEL_RATE_MOVING = 120.0f;
constexpr float MIN_STEERING_AUTHORITY = 30.0f;

constexpr unsigned long DIAGNOSTIC_INTERVAL_MS = 100;
constexpr unsigned long READY_SWITCH_DEBOUNCE_MS = 30;
constexpr unsigned long INACTIVITY_LOCK_MS = 120000;
constexpr uint8_t STATUS_LED_RED = 64;
constexpr uint8_t STATUS_LED_GREEN = 64;
constexpr uint8_t STATUS_LED_AMBER_RED = 64;
constexpr uint8_t STATUS_LED_AMBER_GREEN = 32;

float applyDeadband(float value)
{
    if (fabs(value) <= DEADBAND)
    {
        return 0.0f;
    }

    if (value > 0.0f)
    {
        return (value - DEADBAND) * 100.0f / (100.0f - DEADBAND);
    }

    return (value + DEADBAND) * 100.0f / (100.0f - DEADBAND);
}

float normalizeAxis(int raw, int minVal, int centerVal, int maxVal)
{
    if (raw < centerVal)
    {
        return -100.0f * (centerVal - raw) /
            static_cast<float>(centerVal - minVal);
    }

    return 100.0f * (raw - centerVal) /
        static_cast<float>(maxVal - centerVal);
}

float applyResponseCurve(float value)
{
    static const float input[] = {
        0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
    };

    static const float output[] = {
        0, 2, 6, 12, 20, 30, 42, 56, 70, 85, 100
    };

    float sign = (value < 0.0f) ? -1.0f : 1.0f;
    float v = fabs(value);

    for (int i = 0; i < 10; i++)
    {
        if (v <= input[i + 1])
        {
            float t = (v - input[i]) /
                (input[i + 1] - input[i]);

            float result = output[i] +
                t * (output[i + 1] - output[i]);

            return sign * result;
        }
    }

    return sign * 100.0f;
}

float applySlewRate(
    float target,
    float current,
    float dt,
    float accelerationRate,
    float decelerationRate
)
{
    float maxDeceleration = decelerationRate * dt;

    // During a direction reversal, decelerate to zero first without
    // allowing the output to cross zero in this update.
    if ((target > 0.0f && current < 0.0f) ||
        (target < 0.0f && current > 0.0f))
    {
        if (fabs(current) <= maxDeceleration)
        {
            return 0.0f;
        }

        return current - copysign(maxDeceleration, current);
    }

    bool accelerating = fabs(target) > fabs(current);
    float rate = accelerating ? accelerationRate : decelerationRate;
    float maxChange = rate * dt;

    float difference = target - current;

    if (difference > maxChange)
    {
        difference = maxChange;
    }
    else if (difference < -maxChange)
    {
        difference = -maxChange;
    }

    return current + difference;
}

uint32_t mapDriveToCrsfChannel(float drive)
{
    drive = constrain(drive, -100.0f, 100.0f);

    if (drive < 0.0f)
    {
        const float fraction = (drive + 100.0f) / 100.0f;
        return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_MIN +
            fraction * (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN));
    }

    const float fraction = drive / 100.0f;
    return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_MID +
        fraction * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID));
}
}

void TorqueDollyHandset::Begin()
{
    pinMode(PIN_READY_SWITCH, INPUT_PULLUP);
    neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, 0);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_X, ADC_11db);
    analogSetPinAttenuation(PIN_Y, ADC_11db);

    // Present a controller connection so the normal TX startup state machine
    // enables the RF scheduler without requiring an external CRSF handset.
    if (connected)
    {
        connected();
    }
}

void TorqueDollyHandset::End()
{
}

void TorqueDollyHandset::handleInput()
{
    int xRaw = analogRead(PIN_X);
    int yRaw = analogRead(PIN_Y);

    static bool filterInitialized = false;
    static float xFiltered = 0.0f;
    static float yFiltered = 0.0f;

    if (!filterInitialized)
    {
        xFiltered = xRaw;
        yFiltered = yRaw;
        filterInitialized = true;
    }
    else
    {
        xFiltered = EMA_ALPHA * xRaw +
            (1.0f - EMA_ALPHA) * xFiltered;
        yFiltered = EMA_ALPHA * yRaw +
            (1.0f - EMA_ALPHA) * yFiltered;
    }

    float xNormalized = normalizeAxis(xFiltered, 400, 1600, 2860);
    float yNormalized = normalizeAxis(yFiltered, 390, 1600, 2860);

    float x = applyDeadband(xNormalized);
    float y = applyDeadband(yNormalized);

    x = applyResponseCurve(x);
    y = applyResponseCurve(y);

    enum class ReadyState
    {
        Locked,
        Ready,
        ControlledStop
    };

    static ReadyState readyState = ReadyState::Locked;
    static bool switchInitialized = false;
    static bool switchRawReady = false;
    static bool switchStableReady = false;
    static unsigned long switchChangedAt = 0;
    static bool readyTransitionArmed = false;
    static bool startupNeutral = true;
    static unsigned long inactivityStartedAt = 0;
    static bool inactivityLocked = false;

    auto updateStatusLed = [](
        ReadyState state,
        bool isInactivityLocked,
        unsigned long now
    ) {
        static bool initialized = false;
        static ReadyState lastState;
        static bool lastInactivityLocked = false;
        static bool lastInactivityRed = false;

        bool inactivityRed =
            isInactivityLocked &&
            state == ReadyState::Locked &&
            now % 1000 < 100;

        if (
            initialized &&
            state == lastState &&
            isInactivityLocked == lastInactivityLocked &&
            (!isInactivityLocked || inactivityRed == lastInactivityRed)
        )
        {
            return;
        }

        initialized = true;
        lastState = state;
        lastInactivityLocked = isInactivityLocked;
        lastInactivityRed = inactivityRed;

        if (isInactivityLocked && state == ReadyState::Locked)
        {
            if (inactivityRed)
            {
                neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, 0);
            }
            else
            {
                neopixelWrite(PIN_STATUS_LED, 0, STATUS_LED_GREEN, 0);
            }
            return;
        }

        switch (state)
        {
            case ReadyState::Locked:
                neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, 0);
                break;
            case ReadyState::Ready:
                neopixelWrite(PIN_STATUS_LED, 0, STATUS_LED_GREEN, 0);
                break;
            case ReadyState::ControlledStop:
                neopixelWrite(
                    PIN_STATUS_LED,
                    STATUS_LED_AMBER_RED,
                    STATUS_LED_AMBER_GREEN,
                    0
                );
                break;
        }
    };

    bool switchReady = digitalRead(PIN_READY_SWITCH) == HIGH;
    unsigned long nowMillis = millis();

    if (!switchInitialized)
    {
        switchInitialized = true;
        switchRawReady = switchReady;
        switchStableReady = switchReady;
        switchChangedAt = nowMillis;
        readyTransitionArmed = !switchStableReady;
    }
    else if (switchReady != switchRawReady)
    {
        switchRawReady = switchReady;
        switchChangedAt = nowMillis;
    }
    else if (
        switchStableReady != switchRawReady &&
        nowMillis - switchChangedAt >= READY_SWITCH_DEBOUNCE_MS
    )
    {
        bool wasStableReady = switchStableReady;
        switchStableReady = switchRawReady;

        if (!switchStableReady)
        {
            inactivityLocked = false;

            if (readyState == ReadyState::Ready)
            {
                readyState = ReadyState::ControlledStop;
                readyTransitionArmed = false;
            }
            else if (readyState == ReadyState::Locked)
            {
                readyTransitionArmed = true;
            }
        }
        else if (
            !wasStableReady &&
            readyState == ReadyState::Locked &&
            readyTransitionArmed &&
            fabs(xNormalized) <= DEADBAND &&
            fabs(yNormalized) <= DEADBAND &&
            !startupNeutral
        )
        {
            readyState = ReadyState::Ready;
            readyTransitionArmed = false;
            inactivityLocked = false;
            inactivityStartedAt = nowMillis;
        }
    }

    static float xDrive = 0.0f;
    static float yDrive = 0.0f;
    static unsigned long lastUpdate = millis();

    unsigned long now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    if (dt > 0.1f)
    {
        dt = 0.1f;
    }

    bool startupNeutralHold = startupNeutral;

    if (startupNeutral)
    {
        if (
            fabs(xNormalized) <= DEADBAND &&
            fabs(yNormalized) <= DEADBAND
        )
        {
            startupNeutral = false;
            xDrive = 0.0f;
            yDrive = 0.0f;
            lastUpdate = now;
        }
    }

    if (startupNeutralHold)
    {
        xDrive = 0.0f;
        yDrive = 0.0f;
    }

    if (readyState == ReadyState::Ready)
    {
        bool joystickNeutral =
            fabs(xNormalized) <= DEADBAND &&
            fabs(yNormalized) <= DEADBAND;

        if (!joystickNeutral)
        {
            inactivityStartedAt = now;
        }
        else if (
            inactivityStartedAt != 0 &&
            now - inactivityStartedAt >= INACTIVITY_LOCK_MS
        )
        {
            readyState = ReadyState::Locked;
            readyTransitionArmed = false;
            inactivityLocked = true;
            inactivityStartedAt = 0;
        }
    }

    if (readyState != ReadyState::Ready)
    {
        x = 0.0f;
        y = 0.0f;
    }

    float yDecelerationRate = DECEL_RATE_GENTLE;
    float yCurrentMagnitude = fabs(yDrive);
    float yTargetMagnitude = fabs(y);
    bool yDirectionReversal =
        (y > 0.0f && yDrive < 0.0f) ||
        (y < 0.0f && yDrive > 0.0f);

    if (yDirectionReversal)
    {
        yDecelerationRate = DECEL_RATE_STOP;
    }
    else if (
        yCurrentMagnitude > 0.0001f &&
        yTargetMagnitude < yCurrentMagnitude
    )
    {
        float releaseFraction =
            (yCurrentMagnitude - yTargetMagnitude) /
            yCurrentMagnitude;

        releaseFraction = constrain(releaseFraction, 0.0f, 1.0f);

        yDecelerationRate =
            DECEL_RATE_GENTLE +
            (DECEL_RATE_STOP - DECEL_RATE_GENTLE) *
            releaseFraction;
    }

    if (!startupNeutralHold)
    {
        yDrive = applySlewRate(
            y,
            yDrive,
            dt,
            ACCEL_RATE,
            yDecelerationRate
        );
    }

    float speedMagnitude = fabs(yDrive);
    float steeringRateBlend =
        min(speedMagnitude, PIVOT_BLEND_END) /
        PIVOT_BLEND_END;

    float steeringAccelerationRate =
        STEERING_ACCEL_RATE_STANDSTILL +
        (STEERING_ACCEL_RATE_MOVING -
         STEERING_ACCEL_RATE_STANDSTILL) *
        steeringRateBlend;

    float steeringDecelerationRate =
        STEERING_DECEL_RATE_STANDSTILL +
        (STEERING_DECEL_RATE_MOVING -
         STEERING_DECEL_RATE_STANDSTILL) *
        steeringRateBlend;

    if (!startupNeutralHold)
    {
        xDrive = applySlewRate(
            x,
            xDrive,
            dt,
            steeringAccelerationRate,
            steeringDecelerationRate
        );
    }

    if (
        readyState == ReadyState::ControlledStop &&
        fabs(xDrive) <= 0.0001f &&
        fabs(yDrive) <= 0.0001f
    )
    {
        xDrive = 0.0f;
        yDrive = 0.0f;
        readyState = ReadyState::Locked;
        readyTransitionArmed = !switchStableReady;
        inactivityLocked = false;
        inactivityStartedAt = 0;
    }

    updateStatusLed(readyState, inactivityLocked, now);

    float lowSpeedSteeringLimit =
        speedMagnitude >= PIVOT_BLEND_END
            ? FULL_STEERING_AUTHORITY
            : PIVOT_TURN_MAX +
                  (FULL_STEERING_AUTHORITY - PIVOT_TURN_MAX) *
                  (speedMagnitude / PIVOT_BLEND_END);

    float highSpeedSteeringAuthority =
        FULL_STEERING_AUTHORITY -
        (FULL_STEERING_AUTHORITY - MIN_STEERING_AUTHORITY) *
        (speedMagnitude / 100.0f);

    float steeringAuthority = min(
        lowSpeedSteeringLimit,
        highSpeedSteeringAuthority
    );

    float effectiveX =
        xDrive * steeringAuthority / 100.0f;

    float leftDrive = yDrive + effectiveX;
    float rightDrive = yDrive - effectiveX;

    float maxMixedMagnitude = max(
        fabs(leftDrive),
        fabs(rightDrive)
    );

    if (maxMixedMagnitude > 100.0f)
    {
        float scale = 100.0f / maxMixedMagnitude;
        leftDrive *= scale;
        rightDrive *= scale;
    }

    uint32_t leftChannel = mapDriveToCrsfChannel(leftDrive);
    uint32_t rightChannel = mapDriveToCrsfChannel(rightDrive);

    for (uint32_t &channel : ChannelData)
    {
        channel = CRSF_CHANNEL_VALUE_MID;
    }

    ChannelData[0] = leftChannel;
    ChannelData[1] = rightChannel;
    RCdataLastRecv = micros();

    static unsigned long lastDiagnostic = 0;
    if (now - lastDiagnostic >= DIAGNOSTIC_INTERVAL_MS)
    {
        lastDiagnostic = now;
        Serial.printf(
            "X_RAW: %4d Y_RAW: %4d X_FILTERED: %7.2f "
            "Y_FILTERED: %7.2f X_CMD: %7.2f%% X_DRIVE: %7.2f%% "
            "Y_CMD: %7.2f%% Y_DRIVE: %7.2f%% "
            "STEER_AUTHORITY: %7.2f%% EFFECTIVE_X: %7.2f%% "
            "LEFT: %7.2f%% RIGHT: %7.2f%% CH1: %lu CH2: %lu\n",
            xRaw,
            yRaw,
            xFiltered,
            yFiltered,
            x,
            xDrive,
            y,
            yDrive,
            steeringAuthority,
            effectiveX,
            leftDrive,
            rightDrive,
            static_cast<unsigned long>(leftChannel),
            static_cast<unsigned long>(rightChannel)
        );
    }
}

#endif
