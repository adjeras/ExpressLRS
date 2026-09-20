#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_TX) && defined(TORQUE_DOLLY_HANDSET)

#include "TorqueDollyHandset.h"
#include "crsf_protocol.h"

namespace
{
constexpr int PIN_JOY_X = 1;
constexpr int PIN_JOY_Y = 2;

constexpr int X_MIN = 0;
constexpr int X_CENTER = 1634;
constexpr int X_MAX = 3431;
constexpr int Y_MIN = 0;
constexpr int Y_CENTER = 1635;
constexpr int Y_MAX = 3431;

constexpr float ACCEL_RATE = 50.0f;
constexpr float DECEL_RATE = 200.0f;
constexpr float DEADBAND = 2.0f;
constexpr float EMA_ALPHA = 0.20f;
constexpr float MIN_STEERING_AUTHORITY = 30.0f;

float normalizeAxis(float raw, int minVal, int centerVal, int maxVal)
{
    float value;
    if (raw >= centerVal)
    {
        value = 100.0f * (raw - centerVal) / (maxVal - centerVal);
    }
    else
    {
        value = 100.0f * (raw - centerVal) / (centerVal - minVal);
    }
    return constrain(value, -100.0f, 100.0f);
}

float applyDeadband(float value)
{
    if (fabsf(value) <= DEADBAND)
    {
        return 0.0f;
    }
    if (value > 0.0f)
    {
        return (value - DEADBAND) * 100.0f / (100.0f - DEADBAND);
    }
    return (value + DEADBAND) * 100.0f / (100.0f - DEADBAND);
}

float applyResponseCurve(float value)
{
    static constexpr float input[] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
    static constexpr float output[] = {0, 2, 6, 12, 20, 30, 42, 56, 70, 85, 100};

    const float sign = value < 0.0f ? -1.0f : 1.0f;
    const float magnitude = fabsf(value);
    for (int i = 0; i < 10; ++i)
    {
        if (magnitude <= input[i + 1])
        {
            const float t = (magnitude - input[i]) / (input[i + 1] - input[i]);
            return sign * (output[i] + t * (output[i + 1] - output[i]));
        }
    }
    return sign * 100.0f;
}

float applySlewRate(float target, float current, float dt)
{
    const float maxDeceleration = DECEL_RATE * dt;
    if ((target > 0.0f && current < 0.0f) ||
        (target < 0.0f && current > 0.0f))
    {
        if (fabsf(current) <= maxDeceleration)
        {
            return 0.0f;
        }
        return current - copysignf(maxDeceleration, current);
    }

    const bool accelerating = fabsf(target) > fabsf(current);
    const float maxChange = (accelerating ? ACCEL_RATE : DECEL_RATE) * dt;
    float difference = target - current;
    difference = constrain(difference, -maxChange, maxChange);
    return current + difference;
}

uint32_t mapDriveToCrsfChannel(float drive)
{
    drive = constrain(drive, -100.0f, 100.0f);
    if (drive < 0.0f)
    {
        const float fraction = (drive + 100.0f) / 100.0f;
        return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_STD_MIN +
            fraction * (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_STD_MIN));
    }

    const float fraction = drive / 100.0f;
    return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_MID +
        fraction * (CRSF_CHANNEL_VALUE_STD_MAX - CRSF_CHANNEL_VALUE_MID));
}
}

void TorqueDollyHandset::Begin()
{
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_JOY_X, ADC_11db);
    analogSetPinAttenuation(PIN_JOY_Y, ADC_11db);

    xFiltered = 0.0f;
    yFiltered = 0.0f;
    xDrive = 0.0f;
    yDrive = 0.0f;
    lastUpdate = millis();
    startupNeutral = true;
    filterInitialized = false;

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
    static uint32_t lastDiagnosticMs = 0;

    const int xRaw = analogRead(PIN_JOY_X);
    const int yRaw = analogRead(PIN_JOY_Y);

    if (!filterInitialized)
    {
        xFiltered = xRaw;
        yFiltered = yRaw;
        filterInitialized = true;
    }
    else
    {
        xFiltered = EMA_ALPHA * xRaw + (1.0f - EMA_ALPHA) * xFiltered;
        yFiltered = EMA_ALPHA * yRaw + (1.0f - EMA_ALPHA) * yFiltered;
    }

    const float xNormalized = normalizeAxis(xFiltered, X_MIN, X_CENTER, X_MAX);
    const float yNormalized = normalizeAxis(yFiltered, Y_MIN, Y_CENTER, Y_MAX);
    const bool diagnosticStartup = startupNeutral;

    const auto printDiagnostic = [&](float leftDrive, float rightDrive,
                                     uint32_t ch1, uint32_t ch2) {
        const uint32_t now = millis();
        if (now - lastDiagnosticMs >= 200)
        {
            USBSerial.printf(
                "RAW X=%d Y=%d NORM X=%.2f Y=%.2f DRIVE X=%.2f Y=%.2f "
                "LEFT=%.2f RIGHT=%.2f CH1=%u CH2=%u STARTUP=%d\n",
                xRaw, yRaw, xNormalized, yNormalized, xDrive, yDrive,
                leftDrive, rightDrive, ch1, ch2, diagnosticStartup ? 1 : 0);
            lastDiagnosticMs = now;
        }
    };

    if (startupNeutral)
    {
        if (fabsf(xNormalized) <= DEADBAND && fabsf(yNormalized) <= DEADBAND)
        {
            startupNeutral = false;
            xDrive = 0.0f;
            yDrive = 0.0f;
            lastUpdate = millis();
        }

        uint32_t localChannelData[CRSF_NUM_CHANNELS];
        for (unsigned i = 0; i < CRSF_NUM_CHANNELS; ++i)
        {
            localChannelData[i] = CRSF_CHANNEL_VALUE_MID;
        }
        PerformChannelOverrides(localChannelData, CRSF_NUM_CHANNELS);
        localChannelData[0] = CRSF_CHANNEL_VALUE_MID;
        localChannelData[1] = CRSF_CHANNEL_VALUE_MID;
        printDiagnostic(0.0f, 0.0f, localChannelData[0], localChannelData[1]);
        RCDataReceived(localChannelData, CRSF_NUM_CHANNELS);
        return;
    }

    const float xCommand = applyResponseCurve(applyDeadband(xNormalized));
    const float yCommand = applyResponseCurve(applyDeadband(yNormalized));

    const uint32_t now = millis();
    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;
    if (dt > 0.1f)
    {
        dt = 0.1f;
    }

    xDrive = applySlewRate(xCommand, xDrive, dt);
    yDrive = applySlewRate(yCommand, yDrive, dt);

    const float steeringAuthority = 100.0f -
        (100.0f - MIN_STEERING_AUTHORITY) * (fabsf(yDrive) / 100.0f);
    const float effectiveX = xDrive * steeringAuthority / 100.0f;

    float leftDrive = yDrive + effectiveX;
    float rightDrive = yDrive - effectiveX;
    const float maxMixedMagnitude =
        fabsf(leftDrive) > fabsf(rightDrive) ? fabsf(leftDrive) : fabsf(rightDrive);
    if (maxMixedMagnitude > 100.0f)
    {
        const float scale = 100.0f / maxMixedMagnitude;
        leftDrive *= scale;
        rightDrive *= scale;
    }

    uint32_t localChannelData[CRSF_NUM_CHANNELS];
    for (unsigned i = 0; i < CRSF_NUM_CHANNELS; ++i)
    {
        localChannelData[i] = CRSF_CHANNEL_VALUE_MID;
    }
    localChannelData[0] = mapDriveToCrsfChannel(leftDrive);
    localChannelData[1] = mapDriveToCrsfChannel(rightDrive);

    PerformChannelOverrides(localChannelData, CRSF_NUM_CHANNELS);
    printDiagnostic(leftDrive, rightDrive, localChannelData[0], localChannelData[1]);
    RCDataReceived(localChannelData, CRSF_NUM_CHANNELS);
}

#endif
