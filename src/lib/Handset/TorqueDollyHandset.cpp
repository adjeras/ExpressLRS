#include "targets.h"

#if defined(PLATFORM_ESP32) && defined(TARGET_TX) && defined(TORQUE_DOLLY_HANDSET)

#include "TorqueDollyHandset.h"
#include "TorqueDollyControlParameters.h"
#include "crsf_protocol.h"
#include "common.h"
#include "timingDiagnostics.h"
#include <Preferences.h>

#if defined(TORQUE_DOLLY_BLE_TELEMETRY)
namespace
{
portMUX_TYPE diagnosticStateMux = portMUX_INITIALIZER_UNLOCKED;
TorqueDollyDiagnosticState diagnosticState = TorqueDollyDiagnosticState::Locked;
portMUX_TYPE joystickSnapshotMux = portMUX_INITIALIZER_UNLOCKED;
TorqueDollyJoystickSnapshot joystickSnapshot = {};
}
#endif

namespace
{
using namespace TorqueDollyControlParameters;

Preferences autoPowerOffPreferences;
portMUX_TYPE autoPowerOffStateMux = portMUX_INITIALIZER_UNLOCKED;
uint8_t autoPowerOffMinutes = DEFAULT_AUTO_POWER_OFF_MINUTES;
volatile bool autoPowerOffRequest = false;
volatile uint32_t autoPowerOffGeneration = 0;

void loadAutoPowerOffMinutes()
{
    if (!autoPowerOffPreferences.begin("td_power", false))
    {
        return;
    }
    const uint8_t saved = autoPowerOffPreferences.getUChar(
        "minutes", DEFAULT_AUTO_POWER_OFF_MINUTES);
    portENTER_CRITICAL(&autoPowerOffStateMux);
    autoPowerOffMinutes = saved >= AUTO_POWER_OFF_MINUTES_MIN &&
        saved <= AUTO_POWER_OFF_MINUTES_MAX
        ? saved : DEFAULT_AUTO_POWER_OFF_MINUTES;
    autoPowerOffGeneration++;
    portEXIT_CRITICAL(&autoPowerOffStateMux);
    autoPowerOffPreferences.end();
}

float applyDeadband(float value)
{
    if (fabs(value) <= DEADBAND)
    {
        return 0.0f;
    }

    if (value > 0.0f)
    {
        return (value - DEADBAND) * DRIVE_PERCENT_MAX /
            (DRIVE_PERCENT_MAX - DEADBAND);
    }

    return (value + DEADBAND) * DRIVE_PERCENT_MAX /
        (DRIVE_PERCENT_MAX - DEADBAND);
}

float normalizeAxis(int raw, int minVal, float centerVal, int maxVal)
{
    if (raw < centerVal)
    {
        return DRIVE_PERCENT_MIN * (centerVal - raw) /
            static_cast<float>(centerVal - minVal);
    }

    return DRIVE_PERCENT_MAX * (raw - centerVal) /
        static_cast<float>(maxVal - centerVal);
}

float applyResponseCurve(float value)
{
    const auto &input = RESPONSE_CURVE_INPUT;
    const auto &output = RESPONSE_CURVE_OUTPUT;

    float sign = (value < 0.0f) ? -1.0f : 1.0f;
    float v = fabs(value);

    for (int i = 0; i < RESPONSE_CURVE_SEGMENT_COUNT; i++)
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

    return sign * DRIVE_PERCENT_MAX;
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
    drive = constrain(
        drive,
        DRIVE_PERCENT_MIN,
        DRIVE_PERCENT_MAX
    );

    if (drive < 0.0f)
    {
        const float fraction =
            (drive - DRIVE_PERCENT_MIN) / DRIVE_PERCENT_RANGE;
        return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_MIN +
            fraction * (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN));
    }

    const float fraction = drive / DRIVE_PERCENT_RANGE;
    return static_cast<uint32_t>(CRSF_CHANNEL_VALUE_MID +
        fraction * (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID));
}
}

uint8_t TorqueDollyHandset::getAutoPowerOffMinutes()
{
    return autoPowerOffMinutes;
}

bool TorqueDollyHandset::setAutoPowerOffMinutes(uint8_t minutes)
{
    if (minutes < AUTO_POWER_OFF_MINUTES_MIN ||
        minutes > AUTO_POWER_OFF_MINUTES_MAX ||
        !autoPowerOffPreferences.begin("td_power", false))
    {
        return false;
    }

    const size_t written = autoPowerOffPreferences.putUChar("minutes", minutes);
    const bool writeSucceeded = written == 1 &&
        autoPowerOffPreferences.getUChar("minutes", 0) == minutes;
    autoPowerOffPreferences.end();
    if (!writeSucceeded)
    {
        return false;
    }

    portENTER_CRITICAL(&autoPowerOffStateMux);
    autoPowerOffMinutes = minutes;
    autoPowerOffGeneration++;
    portEXIT_CRITICAL(&autoPowerOffStateMux);
    return true;
}

bool TorqueDollyHandset::consumeAutoPowerOffRequest()
{
    portENTER_CRITICAL(&autoPowerOffStateMux);
    const bool requested = autoPowerOffRequest;
    autoPowerOffRequest = false;
    portEXIT_CRITICAL(&autoPowerOffStateMux);
    return requested;
}

bool TorqueDollyHandset::isAutoPowerOffSuspended()
{
    return torqueDollyPowerButtonHeld() || torqueDollyShutdownActive;
}

void TorqueDollyHandset::Begin()
{
    loadAutoPowerOffMinutes();
    pinMode(PIN_READY_SWITCH, INPUT_PULLUP);
    neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, 0);

    analogReadResolution(ADC_RESOLUTION_BITS);
    analogSetPinAttenuation(PIN_X, ADC_ATTENUATION);
    analogSetPinAttenuation(PIN_Y, ADC_ATTENUATION);

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
    timingDiagnosticsRecordJoystickHandler();

    enum class StartupCalibrationState
    {
        Collecting,
        RetryDelay,
        Succeeded
    };

    static StartupCalibrationState calibrationState =
        StartupCalibrationState::Collecting;
    static unsigned long calibrationStartedAt = 0;
    static unsigned long calibrationRetryStartedAt = 0;
    static unsigned long calibrationSampleCount = 0;
    static unsigned long calibrationValidSampleCount = 0;
    static unsigned long calibrationInvalidSampleCount = 0;
    static float calibrationXSum = 0.0f;
    static float calibrationYSum = 0.0f;
    static float calibrationXSumSquares = 0.0f;
    static float calibrationYSumSquares = 0.0f;
    static int calibrationXMin = 0;
    static int calibrationXMax = 0;
    static int calibrationYMin = 0;
    static int calibrationYMax = 0;
    static constexpr unsigned long calibrationWindowCount = 10;
    static constexpr unsigned long calibrationWindowDurationMs = 50;
    static unsigned long calibrationWindowIndex = 0;
    static unsigned long calibrationWindowDuration[calibrationWindowCount] = {};
    static unsigned long calibrationWindowValidSamples[calibrationWindowCount] = {};
    static unsigned long calibrationWindowInvalidSamples[calibrationWindowCount] = {};
    static float calibrationWindowXSum[calibrationWindowCount] = {};
    static float calibrationWindowYSum[calibrationWindowCount] = {};
    static float calibrationWindowXSumSquares[calibrationWindowCount] = {};
    static float calibrationWindowYSumSquares[calibrationWindowCount] = {};
    static int calibrationWindowXMin[calibrationWindowCount] = {};
    static int calibrationWindowXMax[calibrationWindowCount] = {};
    static int calibrationWindowYMin[calibrationWindowCount] = {};
    static int calibrationWindowYMax[calibrationWindowCount] = {};
    static bool calibrationWindowComplete[calibrationWindowCount] = {};
    static float runtimeXCenter = X_CENTER;
    static float runtimeYCenter = Y_CENTER;

    int xRaw = analogRead(PIN_X);
    int yRaw = analogRead(PIN_Y);
    unsigned long now = millis();

    static bool filterInitialized = false;
    static float xFiltered = 0.0f;
    static float yFiltered = 0.0f;

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
    static unsigned long activityQualificationStartedAt = 0;
    static unsigned long inactivityResetCount = 0;
    static bool inactivityLocked = false;
    static unsigned long autoPowerOffStartedAt = 0;
    static unsigned long autoPowerOffActivityCandidateStartedAt = 0;
    static bool autoPowerOffActivityQualified = false;
    static unsigned long calibrationTimeoutStartedAt = 0;
    static uint32_t observedAutoPowerOffGeneration = 0;
    static unsigned long lastAutoPowerOffDiagnosticAt = 0;
    static unsigned long autoPowerOffNonNeutralResetCount = 0;
    static unsigned long autoPowerOffSuspensionResetCount = 0;
    static unsigned long autoPowerOffGenerationChangeCount = 0;
    static unsigned long autoPowerOffNonNeutralResetReportCount = 0;
    static unsigned long autoPowerOffSuspensionResetReportCount = 0;
    static unsigned long autoPowerOffGenerationChangeReportCount = 0;
#if defined(TORQUE_DOLLY_INACTIVITY_VISUAL_DIAGNOSTICS)
    static unsigned long magentaVisibleUntil = 0;
#endif

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

    float xNormalized = normalizeAxis(
        xFiltered, X_MIN, runtimeXCenter, X_MAX);
    float yNormalized = normalizeAxis(
        yFiltered, Y_MIN, runtimeYCenter, Y_MAX);

    float x = applyDeadband(xNormalized);
    float y = applyDeadband(yNormalized);

    x = applyResponseCurve(x);
    y = applyResponseCurve(y);

    auto updateStatusLed = [](
        ReadyState state,
        bool isInactivityLocked,
        unsigned long now,
        bool visualDiagnosticActive,
        bool calibrationComplete,
        bool calibrationFailed
    ) {
        static bool initialized = false;
#if defined(TORQUE_DOLLY_RF_TEST) && defined(TORQUE_DOLLY_HANDSET) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)
        static bool lastButtonFeedbackActive = false;
        static bool lastButtonFeedbackOn = false;
#endif
        static ReadyState lastState;
        static bool lastInactivityLocked = false;
        static bool lastInactivityRed = false;
        static bool lastVisualDiagnosticActive = false;
        static bool lastCalibrationComplete = false;
        static bool lastCalibrationFailed = false;

        bool inactivityRed =
            isInactivityLocked &&
            state == ReadyState::Locked &&
            now % INACTIVITY_BLINK_PERIOD_MS < INACTIVITY_BLINK_RED_MS;
#if defined(TORQUE_DOLLY_RF_TEST) && defined(TORQUE_DOLLY_HANDSET) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)
        const TorqueDollyButtonFeedbackSnapshot buttonFeedback =
            getTorqueDollyButtonFeedback();
        const bool buttonFeedbackOn = !buttonFeedback.pending ||
            static_cast<uint32_t>(now - buttonFeedback.blinkStartedAtMs) %
                BUTTON_FEEDBACK_BLINK_PERIOD_MS < BUTTON_FEEDBACK_BLINK_ON_MS;

        if (buttonFeedback.active)
        {
            if (!lastButtonFeedbackActive ||
                buttonFeedbackOn != lastButtonFeedbackOn)
            {
                neopixelWrite(
                    PIN_STATUS_LED,
                    buttonFeedbackOn ? STATUS_LED_PINK_RED : 0,
                    0,
                    buttonFeedbackOn ? STATUS_LED_PINK_BLUE : 0
                );
                lastButtonFeedbackOn = buttonFeedbackOn;
            }
            lastButtonFeedbackActive = true;
            return;
        }

        const bool feedbackJustEnded = lastButtonFeedbackActive;
        lastButtonFeedbackActive = false;
        lastButtonFeedbackOn = false;
#else
        const bool feedbackJustEnded = false;
#endif

        if (
            initialized &&
            !feedbackJustEnded &&
            calibrationComplete &&
            state == lastState &&
            isInactivityLocked == lastInactivityLocked &&
            (!isInactivityLocked || inactivityRed == lastInactivityRed) &&
            visualDiagnosticActive == lastVisualDiagnosticActive
        )
        {
            return;
        }

        initialized = true;
        lastState = state;
        lastInactivityLocked = isInactivityLocked;
        lastInactivityRed = inactivityRed;
        lastVisualDiagnosticActive = visualDiagnosticActive;
        lastCalibrationComplete = calibrationComplete;
        lastCalibrationFailed = calibrationFailed;

        if (!calibrationComplete)
        {
            if (calibrationFailed)
            {
                bool failureRed =
                    now % CALIBRATION_FAILURE_BLINK_PERIOD_MS <
                    CALIBRATION_FAILURE_RED_MS;
                if (failureRed)
                {
                    neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, 0);
                }
                else
                {
                    neopixelWrite(PIN_STATUS_LED, 0, 0, 0);
                }
            }
            else
            {
                neopixelWrite(
                    PIN_STATUS_LED,
                    0,
                    0,
                    STATUS_LED_CALIBRATION_BLUE
                );
            }
            return;
        }

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

#if defined(TORQUE_DOLLY_INACTIVITY_VISUAL_DIAGNOSTICS)
        if (state == ReadyState::Ready && visualDiagnosticActive)
        {
            neopixelWrite(PIN_STATUS_LED, STATUS_LED_RED, 0, STATUS_LED_RED);
            return;
        }
#endif

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
    unsigned long nowMillis = now;
    bool calibrationWasComplete =
        calibrationState == StartupCalibrationState::Succeeded;

    if (calibrationState == StartupCalibrationState::Collecting)
    {
        if (calibrationStartedAt == 0)
        {
            calibrationStartedAt = now;
            calibrationXMin = xRaw;
            calibrationXMax = xRaw;
            calibrationYMin = yRaw;
            calibrationYMax = yRaw;
            calibrationWindowIndex = 0;
            for (unsigned long i = 0; i < calibrationWindowCount; i++)
            {
                calibrationWindowDuration[i] = 0;
                calibrationWindowValidSamples[i] = 0;
                calibrationWindowInvalidSamples[i] = 0;
                calibrationWindowXSum[i] = 0.0f;
                calibrationWindowYSum[i] = 0.0f;
                calibrationWindowXSumSquares[i] = 0.0f;
                calibrationWindowYSumSquares[i] = 0.0f;
                calibrationWindowXMin[i] = 4095;
                calibrationWindowXMax[i] = 0;
                calibrationWindowYMin[i] = 4095;
                calibrationWindowYMax[i] = 0;
                calibrationWindowComplete[i] = false;
            }
        }

        unsigned long calibrationElapsed = now - calibrationStartedAt;
        unsigned long sampleWindowIndex = min(
            calibrationElapsed / calibrationWindowDurationMs,
            calibrationWindowCount - 1
        );
        while (calibrationWindowIndex < sampleWindowIndex)
        {
            calibrationWindowDuration[calibrationWindowIndex] =
                calibrationWindowDurationMs;
            calibrationWindowComplete[calibrationWindowIndex] = true;
            calibrationWindowIndex++;
        }

        calibrationSampleCount++;
        bool validSample =
            xRaw >= 0 && xRaw <= 4095 &&
            yRaw >= 0 && yRaw <= 4095;

        if (validSample)
        {
            calibrationValidSampleCount++;
            calibrationXSum += xRaw;
            calibrationYSum += yRaw;
            calibrationXSumSquares +=
                static_cast<float>(xRaw) * xRaw;
            calibrationYSumSquares +=
                static_cast<float>(yRaw) * yRaw;
            calibrationXMin = min(calibrationXMin, xRaw);
            calibrationXMax = max(calibrationXMax, xRaw);
            calibrationYMin = min(calibrationYMin, yRaw);
            calibrationYMax = max(calibrationYMax, yRaw);

            calibrationWindowValidSamples[sampleWindowIndex]++;
            calibrationWindowXSum[sampleWindowIndex] += xRaw;
            calibrationWindowYSum[sampleWindowIndex] += yRaw;
            calibrationWindowXSumSquares[sampleWindowIndex] +=
                static_cast<float>(xRaw) * xRaw;
            calibrationWindowYSumSquares[sampleWindowIndex] +=
                static_cast<float>(yRaw) * yRaw;
            calibrationWindowXMin[sampleWindowIndex] = min(
                calibrationWindowXMin[sampleWindowIndex], xRaw);
            calibrationWindowXMax[sampleWindowIndex] = max(
                calibrationWindowXMax[sampleWindowIndex], xRaw);
            calibrationWindowYMin[sampleWindowIndex] = min(
                calibrationWindowYMin[sampleWindowIndex], yRaw);
            calibrationWindowYMax[sampleWindowIndex] = max(
                calibrationWindowYMax[sampleWindowIndex], yRaw);
        }
        else
        {
            calibrationInvalidSampleCount++;
            calibrationWindowInvalidSamples[sampleWindowIndex]++;
        }

        if (now - calibrationStartedAt >= STARTUP_CALIBRATION_DURATION_MS)
        {
            if (calibrationWindowIndex < calibrationWindowCount)
            {
                calibrationWindowDuration[calibrationWindowIndex] =
                    calibrationElapsed -
                    calibrationWindowIndex * calibrationWindowDurationMs;
                calibrationWindowComplete[calibrationWindowIndex] = true;
            }

            unsigned long completeWindowCount = 0;
            unsigned long sampledWindowCount = 0;
            float windowMeanX[calibrationWindowCount] = {};
            float windowMeanY[calibrationWindowCount] = {};
            float maxWindowMeanSpanX = 0.0f;
            float maxWindowMeanSpanY = 0.0f;
            float firstToLastDriftX = 0.0f;
            float firstToLastDriftY = 0.0f;
            float maxAdjacentStepX = 0.0f;
            float maxAdjacentStepY = 0.0f;

            for (unsigned long i = 0; i < calibrationWindowCount; i++)
            {
                if (calibrationWindowComplete[i])
                {
                    completeWindowCount++;
                }
                if (calibrationWindowValidSamples[i] > 0)
                {
                    sampledWindowCount++;
                    windowMeanX[i] = calibrationWindowXSum[i] /
                        calibrationWindowValidSamples[i];
                    windowMeanY[i] = calibrationWindowYSum[i] /
                        calibrationWindowValidSamples[i];
                }
            }

            bool allWindowsSampled =
                completeWindowCount == calibrationWindowCount &&
                sampledWindowCount == calibrationWindowCount;
            if (allWindowsSampled)
            {
                float minWindowMeanX = windowMeanX[0];
                float maxWindowMeanX = windowMeanX[0];
                float minWindowMeanY = windowMeanY[0];
                float maxWindowMeanY = windowMeanY[0];
                for (unsigned long i = 1; i < calibrationWindowCount; i++)
                {
                    minWindowMeanX = min(minWindowMeanX, windowMeanX[i]);
                    maxWindowMeanX = max(maxWindowMeanX, windowMeanX[i]);
                    minWindowMeanY = min(minWindowMeanY, windowMeanY[i]);
                    maxWindowMeanY = max(maxWindowMeanY, windowMeanY[i]);
                    maxAdjacentStepX = max(
                        maxAdjacentStepX,
                        fabs(windowMeanX[i] - windowMeanX[i - 1])
                    );
                    maxAdjacentStepY = max(
                        maxAdjacentStepY,
                        fabs(windowMeanY[i] - windowMeanY[i - 1])
                    );
                }
                maxWindowMeanSpanX = maxWindowMeanX - minWindowMeanX;
                maxWindowMeanSpanY = maxWindowMeanY - minWindowMeanY;
                firstToLastDriftX = fabs(
                    windowMeanX[calibrationWindowCount - 1] -
                    windowMeanX[0]
                );
                firstToLastDriftY = fabs(
                    windowMeanY[calibrationWindowCount - 1] -
                    windowMeanY[0]
                );
            }

            Serial.printf(
                "CALIBRATION_WINDOWS complete=%lu/%lu sampled=%lu/%lu "
                "span=(%.2f,%.2f) drift=(%.2f,%.2f) "
                "maxstep=(%.2f,%.2f)\n",
                completeWindowCount,
                calibrationWindowCount,
                sampledWindowCount,
                calibrationWindowCount,
                maxWindowMeanSpanX,
                maxWindowMeanSpanY,
                firstToLastDriftX,
                firstToLastDriftY,
                maxAdjacentStepX,
                maxAdjacentStepY
            );
            for (unsigned long i = 0; i < calibrationWindowCount; i++)
            {
                unsigned long validSamples =
                    calibrationWindowValidSamples[i];
                float meanX = validSamples > 0
                    ? calibrationWindowXSum[i] / validSamples : 0.0f;
                float meanY = validSamples > 0
                    ? calibrationWindowYSum[i] / validSamples : 0.0f;
                float varianceX = validSamples > 0
                    ? calibrationWindowXSumSquares[i] / validSamples -
                        meanX * meanX : 0.0f;
                float varianceY = validSamples > 0
                    ? calibrationWindowYSumSquares[i] / validSamples -
                        meanY * meanY : 0.0f;
                Serial.printf(
                    "CALIBRATION_WINDOW %lu duration=%lu valid=%lu "
                    "invalid=%lu mean=(%.2f,%.2f) stddev=(%.2f,%.2f) "
                    "min=(%d,%d) max=(%d,%d)\n",
                    i,
                    calibrationWindowDuration[i],
                    validSamples,
                    calibrationWindowInvalidSamples[i],
                    meanX,
                    meanY,
                    sqrtf(max(0.0f, varianceX)),
                    sqrtf(max(0.0f, varianceY)),
                    validSamples > 0 ? calibrationWindowXMin[i] : 0,
                    validSamples > 0 ? calibrationWindowYMin[i] : 0,
                    validSamples > 0 ? calibrationWindowXMax[i] : 0,
                    validSamples > 0 ? calibrationWindowYMax[i] : 0
                );
            }

            constexpr unsigned long minWindowSamples = 5;
            constexpr float maxWindowStdDev = 24.0f;
            constexpr float maxWindowMeanSpan = 24.0f;
            constexpr float maxWindowDrift = 16.0f;
            constexpr float maxWindowStep = 12.0f;

            bool enoughSamples =
                calibrationValidSampleCount >=
                STARTUP_CALIBRATION_MIN_SAMPLES;
            bool invalidRateAcceptable =
                calibrationInvalidSampleCount * 10 <=
                calibrationSampleCount;
            bool windowsComplete = completeWindowCount ==
                calibrationWindowCount;
            bool windowsSampled = sampledWindowCount ==
                calibrationWindowCount;
            bool windowsEnoughSamples = windowsComplete && windowsSampled;
            bool windowsFinite = windowsEnoughSamples;
            bool windowsCenterPlausible = windowsEnoughSamples;
            bool windowsStdDevAcceptable = windowsEnoughSamples;

            float xMeansForMedian[calibrationWindowCount] = {};
            float yMeansForMedian[calibrationWindowCount] = {};
            for (unsigned long i = 0; i < calibrationWindowCount; i++)
            {
                unsigned long validSamples =
                    calibrationWindowValidSamples[i];
                if (validSamples < minWindowSamples)
                {
                    windowsEnoughSamples = false;
                    continue;
                }

                float meanX = calibrationWindowXSum[i] / validSamples;
                float meanY = calibrationWindowYSum[i] / validSamples;
                float varianceX =
                    calibrationWindowXSumSquares[i] / validSamples -
                    meanX * meanX;
                float varianceY =
                    calibrationWindowYSumSquares[i] / validSamples -
                    meanY * meanY;
                float stdDevX = sqrtf(max(0.0f, varianceX));
                float stdDevY = sqrtf(max(0.0f, varianceY));

                if (!isfinite(meanX) || !isfinite(meanY) ||
                    !isfinite(stdDevX) || !isfinite(stdDevY))
                {
                    windowsFinite = false;
                }
                if (
                    fabs(meanX - X_CENTER) >
                        STARTUP_CALIBRATION_CENTER_WINDOW_RAW ||
                    fabs(meanY - Y_CENTER) >
                        STARTUP_CALIBRATION_CENTER_WINDOW_RAW
                )
                {
                    windowsCenterPlausible = false;
                }
                if (stdDevX > maxWindowStdDev ||
                    stdDevY > maxWindowStdDev)
                {
                    windowsStdDevAcceptable = false;
                }

                xMeansForMedian[i] = meanX;
                yMeansForMedian[i] = meanY;
            }

            bool windowMeansStable = windowsEnoughSamples && windowsFinite &&
                windowsCenterPlausible && windowsStdDevAcceptable;

            bool windowSpanAcceptable =
                windowMeansStable &&
                maxWindowMeanSpanX <= maxWindowMeanSpan &&
                maxWindowMeanSpanY <= maxWindowMeanSpan;
            bool windowDriftAcceptable =
                windowMeansStable &&
                firstToLastDriftX <= maxWindowDrift &&
                firstToLastDriftY <= maxWindowDrift;
            bool windowStepAcceptable =
                windowMeansStable &&
                maxAdjacentStepX <= maxWindowStep &&
                maxAdjacentStepY <= maxWindowStep;

            float xMean = 0.0f;
            float yMean = 0.0f;
            if (windowMeansStable)
            {
                for (unsigned long i = 0; i < calibrationWindowCount; i++)
                {
                    for (unsigned long j = i + 1;
                         j < calibrationWindowCount; j++)
                    {
                        if (xMeansForMedian[j] < xMeansForMedian[i])
                        {
                            float value = xMeansForMedian[i];
                            xMeansForMedian[i] = xMeansForMedian[j];
                            xMeansForMedian[j] = value;
                        }
                        if (yMeansForMedian[j] < yMeansForMedian[i])
                        {
                            float value = yMeansForMedian[i];
                            yMeansForMedian[i] = yMeansForMedian[j];
                            yMeansForMedian[j] = value;
                        }
                    }
                }
                xMean = (xMeansForMedian[4] + xMeansForMedian[5]) / 2.0f;
                yMean = (yMeansForMedian[4] + yMeansForMedian[5]) / 2.0f;
            }

            bool finalCenterPlausible =
                windowMeansStable &&
                isfinite(xMean) && isfinite(yMean) &&
                fabs(xMean - X_CENTER) <=
                    STARTUP_CALIBRATION_CENTER_WINDOW_RAW &&
                fabs(yMean - Y_CENTER) <=
                    STARTUP_CALIBRATION_CENTER_WINDOW_RAW;
            bool calibrationAccepted =
                enoughSamples && invalidRateAcceptable &&
                windowsComplete && windowsSampled && windowsEnoughSamples &&
                windowsFinite && windowsCenterPlausible &&
                windowsStdDevAcceptable && windowSpanAcceptable &&
                windowDriftAcceptable && windowStepAcceptable &&
                finalCenterPlausible;

            Serial.printf(
                "CALIBRATION %s samples=%lu invalid=%lu "
                "mean=(%.2f,%.2f) windows="
                "samples:%s invalid:%s center:%s stddev:%s "
                "span:%s drift:%s step:%s finite:%s complete:%s\n",
                calibrationAccepted ? "SUCCESS" : "FAILED",
                calibrationSampleCount,
                calibrationInvalidSampleCount,
                xMean,
                yMean,
                enoughSamples ? "PASS" : "FAIL",
                invalidRateAcceptable ? "PASS" : "FAIL",
                finalCenterPlausible ? "PASS" : "FAIL",
                windowsStdDevAcceptable ? "PASS" : "FAIL",
                windowSpanAcceptable ? "PASS" : "FAIL",
                windowDriftAcceptable ? "PASS" : "FAIL",
                windowStepAcceptable ? "PASS" : "FAIL",
                windowsFinite ? "PASS" : "FAIL",
                windowsComplete && windowsSampled &&
                        windowsEnoughSamples ? "PASS" : "FAIL"
            );

#if defined(TORQUE_DOLLY_JOYSTICK_DIAGNOSTICS)
            // Diagnostic wiring mode never adopts a new center or blocks
            // sampling on the production calibration acceptance checks.
            if (calibrationAccepted)
            {
                Serial.println(
                    "JOYSTICK_DIAGNOSTICS calibration accepted; "
                    "using existing reference centers"
                );
            }
            else
            {
                Serial.println(
                    "JOYSTICK_DIAGNOSTICS calibration rejected; "
                    "using existing reference centers"
                );
            }
            calibrationState = StartupCalibrationState::Succeeded;
            startupNeutral = true;
#else
            if (calibrationAccepted)
            {
                runtimeXCenter = xMean;
                runtimeYCenter = yMean;
                xFiltered = runtimeXCenter;
                yFiltered = runtimeYCenter;
                filterInitialized = true;
                calibrationState = StartupCalibrationState::Succeeded;
                startupNeutral = true;
                calibrationTimeoutStartedAt = 0;
                autoPowerOffStartedAt = now;
                autoPowerOffActivityCandidateStartedAt = 0;
                autoPowerOffActivityQualified = false;
                inactivityStartedAt = now;
                activityQualificationStartedAt = 0;
                activityQualificationStartedAt = 0;
                inactivityLocked = false;
                readyTransitionArmed = false;
            }
            else
            {
                calibrationState = StartupCalibrationState::RetryDelay;
                calibrationRetryStartedAt = now;
            }
#endif
        }
    }
    else if (
        calibrationState == StartupCalibrationState::RetryDelay &&
        now - calibrationRetryStartedAt >=
            STARTUP_CALIBRATION_RETRY_DELAY_MS
    )
    {
        calibrationState = StartupCalibrationState::Collecting;
        calibrationStartedAt = 0;
        calibrationWindowIndex = 0;
        calibrationSampleCount = 0;
        calibrationValidSampleCount = 0;
        calibrationInvalidSampleCount = 0;
        calibrationXSum = 0.0f;
        calibrationYSum = 0.0f;
        calibrationXSumSquares = 0.0f;
        calibrationYSumSquares = 0.0f;
    }

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
            activityQualificationStartedAt = 0;
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
            calibrationState == StartupCalibrationState::Succeeded &&
            calibrationWasComplete &&
            readyTransitionArmed &&
            fabs(xNormalized) <= DEADBAND &&
            fabs(yNormalized) <= DEADBAND &&
            !startupNeutral
        )
        {
            readyState = ReadyState::Ready;
            readyTransitionArmed = false;
            activityQualificationStartedAt = 0;
            inactivityLocked = false;
            inactivityStartedAt = nowMillis;
        }
    }

    static float xDrive = 0.0f;
    static float yDrive = 0.0f;
    static unsigned long lastUpdate = millis();

    // A transition debounced during calibration is not eligible. Once the
    // calibration-completion call has finished, a currently Locked switch
    // position can arm the next deliberate Locked-to-Ready transition.
    if (!calibrationWasComplete &&
        calibrationState == StartupCalibrationState::Succeeded &&
        !switchStableReady)
    {
        readyTransitionArmed = true;
    }

    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    if (dt > MAX_CONTROL_DT_SECONDS)
    {
        dt = MAX_CONTROL_DT_SECONDS;
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

    bool joystickNeutral =
        fabs(xNormalized) <= DEADBAND &&
        fabs(yNormalized) <= DEADBAND;
    const char *readyStateName =
        readyState == ReadyState::Ready ? "Ready" :
        readyState == ReadyState::Locked ? "Locked" : "ControlledStop";

    uint8_t configuredAutoPowerOffMinutes;
    uint32_t configuredAutoPowerOffGeneration;
    bool autoPowerOffRequestSnapshot;
    portENTER_CRITICAL(&autoPowerOffStateMux);
    configuredAutoPowerOffMinutes = autoPowerOffMinutes;
    configuredAutoPowerOffGeneration = autoPowerOffGeneration;
    autoPowerOffRequestSnapshot = autoPowerOffRequest;
    portEXIT_CRITICAL(&autoPowerOffStateMux);

    if (configuredAutoPowerOffGeneration != observedAutoPowerOffGeneration)
    {
        observedAutoPowerOffGeneration = configuredAutoPowerOffGeneration;
        autoPowerOffGenerationChangeCount++;
        autoPowerOffStartedAt = now;
        autoPowerOffActivityCandidateStartedAt = 0;
        autoPowerOffActivityQualified = false;
        calibrationTimeoutStartedAt = now;
    }

    const bool firmwareUpdateActive =
        connectionState == wifiUpdate || connectionState == serialUpdate;
    const bool calibrationTimeoutSuspended =
        torqueDollyPowerButtonHeld() || firmwareUpdateActive;

    if (calibrationState == StartupCalibrationState::Succeeded)
    {
        calibrationTimeoutStartedAt = 0;
    }
    else if (calibrationTimeoutSuspended)
    {
        calibrationTimeoutStartedAt = 0;
    }
    else if (calibrationTimeoutStartedAt == 0)
    {
        calibrationTimeoutStartedAt = now;
    }
    else if (
        static_cast<uint32_t>(now - calibrationTimeoutStartedAt) >=
            static_cast<uint32_t>(configuredAutoPowerOffMinutes) * 60UL * 1000UL)
    {
        portENTER_CRITICAL(&autoPowerOffStateMux);
        autoPowerOffRequest = true;
        portEXIT_CRITICAL(&autoPowerOffStateMux);
        Serial.printf("AUTO_POWER_OFF_REQUEST published reason=calibration_timeout\n");
        calibrationTimeoutStartedAt = now;
    }

    if (calibrationState != StartupCalibrationState::Succeeded)
    {
        autoPowerOffStartedAt = 0;
        autoPowerOffActivityCandidateStartedAt = 0;
        autoPowerOffActivityQualified = false;
    }
    else if (isAutoPowerOffSuspended() ||
        firmwareUpdateActive ||
        connectionState > FAILURE_STATES)
    {
        if (autoPowerOffStartedAt != 0)
        {
            autoPowerOffSuspensionResetCount++;
        }
        autoPowerOffStartedAt = 0;
        autoPowerOffActivityCandidateStartedAt = 0;
        autoPowerOffActivityQualified = false;
    }
    else if (!joystickNeutral)
    {
        if (autoPowerOffActivityCandidateStartedAt == 0)
        {
            autoPowerOffActivityCandidateStartedAt = now;
        }
        else if (
            !autoPowerOffActivityQualified &&
            now - autoPowerOffActivityCandidateStartedAt >=
                ACTIVITY_QUALIFICATION_MS
        )
        {
            autoPowerOffActivityQualified = true;
            autoPowerOffStartedAt = 0;
            autoPowerOffNonNeutralResetCount++;
        }

        if (autoPowerOffActivityQualified)
        {
            autoPowerOffStartedAt = 0;
        }
    }
    else
    {
        const bool wasActivityQualified = autoPowerOffActivityQualified;
        const bool hadActivityCandidate =
            autoPowerOffActivityCandidateStartedAt != 0;
        autoPowerOffActivityCandidateStartedAt = 0;
        autoPowerOffActivityQualified = false;

        if (wasActivityQualified || autoPowerOffStartedAt == 0)
        {
            autoPowerOffStartedAt = now;
        }
        else if (hadActivityCandidate)
        {
            // An unqualified excursion must not alter the existing idle epoch.
        }
    }

    if (
        calibrationState == StartupCalibrationState::Succeeded &&
        !isAutoPowerOffSuspended() &&
        !firmwareUpdateActive &&
        connectionState <= FAILURE_STATES &&
        joystickNeutral &&
        autoPowerOffActivityCandidateStartedAt == 0 &&
        !autoPowerOffActivityQualified &&
        autoPowerOffStartedAt != 0 &&
        (uint32_t)(now - autoPowerOffStartedAt) >=
            static_cast<uint32_t>(configuredAutoPowerOffMinutes) * 60UL * 1000UL)
    {
        portENTER_CRITICAL(&autoPowerOffStateMux);
        autoPowerOffRequest = true;
        portEXIT_CRITICAL(&autoPowerOffStateMux);
        Serial.printf("AUTO_POWER_OFF_REQUEST published elapsed_ms=%lu\n",
            static_cast<unsigned long>(now - autoPowerOffStartedAt));
        autoPowerOffStartedAt = now;
    }

    const bool autoPowerOffTimerRunning = autoPowerOffStartedAt != 0;
    const unsigned long autoPowerOffElapsed = autoPowerOffTimerRunning
        ? now - autoPowerOffStartedAt : 0;
    const bool calibrationTimeoutRunning =
        calibrationTimeoutStartedAt != 0;
    const unsigned long calibrationTimeoutElapsed =
        calibrationTimeoutRunning ? now - calibrationTimeoutStartedAt : 0;
    const char *autoPowerOffReason =
        calibrationState != StartupCalibrationState::Succeeded
            ? "calibration"
            : isAutoPowerOffSuspended()
                ? "button_or_shutdown"
                : connectionState == wifiUpdate || connectionState > FAILURE_STATES
                    ? "connection_state"
                    : !joystickNeutral ? "joystick_non_neutral"
                    : autoPowerOffTimerRunning ? "running" : "idle";
    if (now - lastAutoPowerOffDiagnosticAt >=
        AUTO_POWER_OFF_DIAGNOSTIC_INTERVAL_MS)
    {
        const unsigned long nonNeutralResetsSinceReport =
            autoPowerOffNonNeutralResetCount -
            autoPowerOffNonNeutralResetReportCount;
        const unsigned long suspensionResetsSinceReport =
            autoPowerOffSuspensionResetCount -
            autoPowerOffSuspensionResetReportCount;
        const unsigned long generationChangesSinceReport =
            autoPowerOffGenerationChangeCount -
            autoPowerOffGenerationChangeReportCount;
        Serial.printf(
            "AUTO_POWER_OFF_DIAG minutes=%u neutral=%s "
            "x_norm=%7.2f y_norm=%7.2f calibration=%s "
            "ready_state=%s inactivity_locked=%s "
            "button_held=%s connection_state=%d excluded=%s "
            "timer=%s elapsed_ms=%lu activity_candidate=%s "
            "activity_qualified=%s candidate_ms=%lu "
            "calibration_timer=%s calibration_elapsed_ms=%lu "
            "nonneutral_resets=%lu "
            "suspension_resets=%lu generation_changes=%lu reason=%s "
            "generation=%lu request=%s\n",
            configuredAutoPowerOffMinutes,
            joystickNeutral ? "yes" : "no",
            xNormalized,
            yNormalized,
            calibrationState == StartupCalibrationState::Succeeded
                ? "succeeded" : "not_succeeded",
            readyStateName,
            inactivityLocked ? "yes" : "no",
            torqueDollyPowerButtonHeld() ? "yes" : "no",
            static_cast<int>(connectionState),
            connectionState > FAILURE_STATES ? "yes" : "no",
            autoPowerOffTimerRunning ? "running" : "stopped",
            autoPowerOffElapsed,
            autoPowerOffActivityCandidateStartedAt != 0 ? "yes" : "no",
            autoPowerOffActivityQualified ? "yes" : "no",
            autoPowerOffActivityCandidateStartedAt == 0
                ? 0UL : now - autoPowerOffActivityCandidateStartedAt,
            calibrationTimeoutRunning ? "running" : "stopped",
            calibrationTimeoutElapsed,
            nonNeutralResetsSinceReport,
            suspensionResetsSinceReport,
            generationChangesSinceReport,
            autoPowerOffReason,
            static_cast<unsigned long>(configuredAutoPowerOffGeneration),
            autoPowerOffRequestSnapshot ? "yes" : "no"
        );
        lastAutoPowerOffDiagnosticAt = now;
        autoPowerOffNonNeutralResetReportCount =
            autoPowerOffNonNeutralResetCount;
        autoPowerOffSuspensionResetReportCount =
            autoPowerOffSuspensionResetCount;
        autoPowerOffGenerationChangeReportCount =
            autoPowerOffGenerationChangeCount;
    }

#if defined(TORQUE_DOLLY_INACTIVITY_VISUAL_DIAGNOSTICS)
    if (readyState == ReadyState::Ready &&
        calibrationState == StartupCalibrationState::Succeeded &&
        !joystickNeutral)
    {
        magentaVisibleUntil = now;
    }

    bool visualDiagnosticActive =
        readyState == ReadyState::Ready &&
        now - magentaVisibleUntil < 1000;
#else
    bool visualDiagnosticActive = false;
#endif

    if (readyState == ReadyState::Ready &&
        calibrationState == StartupCalibrationState::Succeeded)
    {
        if (!joystickNeutral)
        {
            if (activityQualificationStartedAt == 0)
            {
                activityQualificationStartedAt = now;
            }
            else if (
                now - activityQualificationStartedAt >=
                    ACTIVITY_QUALIFICATION_MS
            )
            {
                inactivityStartedAt = now;
                inactivityResetCount++;
                activityQualificationStartedAt = 0;
            }
        }
        else
        {
            activityQualificationStartedAt = 0;
        }

        if (joystickNeutral &&
            inactivityStartedAt != 0 &&
            now - inactivityStartedAt >= INACTIVITY_LOCK_MS)
        {
            readyState = ReadyState::Locked;
            readyTransitionArmed = false;
            inactivityLocked = true;
            inactivityStartedAt = 0;
            activityQualificationStartedAt = 0;
        }
    }

#if defined(TORQUE_DOLLY_JOYSTICK_DIAGNOSTICS)
    // Diagnostic wiring mode is observational only: remain locked and keep
    // both output commands neutral regardless of switch or joystick input.
    readyState = ReadyState::Locked;
    inactivityLocked = false;
    inactivityStartedAt = 0;
    activityQualificationStartedAt = 0;
    readyTransitionArmed = false;
#endif

    if (readyState != ReadyState::Ready)
    {
        x = 0.0f;
        y = 0.0f;
    }

    if (calibrationState != StartupCalibrationState::Succeeded)
    {
        x = 0.0f;
        y = 0.0f;
        xDrive = 0.0f;
        yDrive = 0.0f;
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
        activityQualificationStartedAt = 0;
    }

    updateStatusLed(
        readyState,
        inactivityLocked,
        now,
        visualDiagnosticActive,
        calibrationState == StartupCalibrationState::Succeeded,
        calibrationState == StartupCalibrationState::RetryDelay
    );

    float lowSpeedSteeringLimit =
        speedMagnitude >= PIVOT_BLEND_END
            ? FULL_STEERING_AUTHORITY
            : PIVOT_TURN_MAX +
                  (FULL_STEERING_AUTHORITY - PIVOT_TURN_MAX) *
                  (speedMagnitude / PIVOT_BLEND_END);

    float highSpeedSteeringAuthority =
        FULL_STEERING_AUTHORITY -
        (FULL_STEERING_AUTHORITY - MIN_STEERING_AUTHORITY) *
        (speedMagnitude / DRIVE_PERCENT_RANGE);

    float steeringAuthority = min(
        lowSpeedSteeringLimit,
        highSpeedSteeringAuthority
    );

    float effectiveX =
        xDrive * steeringAuthority / DRIVE_PERCENT_RANGE;

    float leftDrive = yDrive + effectiveX;
    float rightDrive = yDrive - effectiveX;

    float maxMixedMagnitude = max(
        fabs(leftDrive),
        fabs(rightDrive)
    );

    if (maxMixedMagnitude > DRIVE_PERCENT_MAX)
    {
        float scale = DRIVE_PERCENT_MAX / maxMixedMagnitude;
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

#if defined(TORQUE_DOLLY_BLE_TELEMETRY)
    portENTER_CRITICAL(&joystickSnapshotMux);
    joystickSnapshot.sequence++;
    joystickSnapshot.xRaw = static_cast<uint16_t>(constrain(xRaw, 0, 65535));
    joystickSnapshot.yRaw = static_cast<uint16_t>(constrain(yRaw, 0, 65535));
    joystickSnapshot.xFiltered = xFiltered;
    joystickSnapshot.yFiltered = yFiltered;
    joystickSnapshot.xCenter = runtimeXCenter;
    joystickSnapshot.yCenter = runtimeYCenter;
    joystickSnapshot.xCommand = x;
    joystickSnapshot.yCommand = y;
    portEXIT_CRITICAL(&joystickSnapshotMux);
#endif

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

#if defined(TORQUE_DOLLY_INACTIVITY_DIAGNOSTICS)
    static unsigned long lastInactivityDiagnostic = 0;
    if (now - lastInactivityDiagnostic >= 1000)
    {
        lastInactivityDiagnostic = now;

        const char *stateName = "Locked";
        if (readyState == ReadyState::Ready)
        {
            stateName = "Ready";
        }
        else if (readyState == ReadyState::ControlledStop)
        {
            stateName = "ControlledStop";
        }

        unsigned long elapsedSinceReset =
            inactivityStartedAt == 0 ? 0 : now - inactivityStartedAt;

        Serial.printf(
            "INACTIVITY_DIAG X_FILTERED: %7.2f Y_FILTERED: %7.2f "
            "X_NORMALIZED: %7.2f%% Y_NORMALIZED: %7.2f%% "
            "NEUTRAL: %s ELAPSED_MS: %lu RESETS: %lu "
            "STATE: %s INACTIVITY_LOCKED: %s\n",
            xFiltered,
            yFiltered,
            xNormalized,
            yNormalized,
            joystickNeutral ? "yes" : "no",
            elapsedSinceReset,
            inactivityResetCount,
            stateName,
            inactivityLocked ? "yes" : "no"
        );
    }
#endif

#if defined(TORQUE_DOLLY_BLE_TELEMETRY)
    TorqueDollyDiagnosticState publishedState =
        TorqueDollyDiagnosticState::Locked;
    switch (readyState)
    {
        case ReadyState::Locked:
            publishedState = TorqueDollyDiagnosticState::Locked;
            break;
        case ReadyState::Ready:
            publishedState = TorqueDollyDiagnosticState::Ready;
            break;
        case ReadyState::ControlledStop:
            publishedState = TorqueDollyDiagnosticState::ControlledStop;
            break;
    }

    portENTER_CRITICAL(&diagnosticStateMux);
    diagnosticState = publishedState;
    portEXIT_CRITICAL(&diagnosticStateMux);
#endif
}

#if defined(TORQUE_DOLLY_BLE_TELEMETRY)
TorqueDollyDiagnosticState TorqueDollyHandset::getDiagnosticState()
{
    portENTER_CRITICAL(&diagnosticStateMux);
    TorqueDollyDiagnosticState state = diagnosticState;
    portEXIT_CRITICAL(&diagnosticStateMux);
    return state;
}

TorqueDollyJoystickSnapshot TorqueDollyHandset::getJoystickSnapshot()
{
    portENTER_CRITICAL(&joystickSnapshotMux);
    TorqueDollyJoystickSnapshot snapshot = joystickSnapshot;
    portEXIT_CRITICAL(&joystickSnapshotMux);
    return snapshot;
}
#endif

#endif
