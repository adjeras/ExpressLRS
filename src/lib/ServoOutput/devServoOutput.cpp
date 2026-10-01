#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "devServoOutput.h"
#include "CRSF.h"
#include "config.h"
#include "helpers.h"
#include "rxtx_intf.h"

static uint8_t SERVO_PINS[PWM_MAX_CHANNELS];
static ServoMgr *servoMgr;
// true when the RX has a new channels packet
static bool newChannelsAvailable;
// Absolute max failsafe time if no update is received, regardless of LQ
static constexpr uint32_t FAILSAFE_ABS_TIMEOUT_MS = 1000U;
static constexpr uint16_t TORQUE_DOLLY_RAMP_RATE_US_PER_SEC = 350U;
static constexpr uint16_t TORQUE_DOLLY_NEUTRAL_TOLERANCE_US = 30U;
static constexpr uint16_t SERVO_FAILSAFE_MIN = 988U;

// Local logical commands are kept here so ServoMgr does not need a new API.
static uint16_t lastLogicalUs[PWM_MAX_CHANNELS];
static bool lastLogicalUsValid[PWM_MAX_CHANNELS];
static uint32_t torqueDollyRampStart;
static uint16_t torqueDollyRampFrom[PWM_MAX_CHANNELS];
static bool torqueDollyRamping;
static bool torqueDollyHolding;
static bool torqueDollyReleaseReady;

void ICACHE_RAM_ATTR servoNewChannelsAvaliable()
{
    newChannelsAvailable = true;
}

uint16_t servoOutputModeToUs(eServoOutputMode mode)
{
    switch (mode)
    {
    case som50Hz:
        return (1000000U / 50U);
    case som60Hz:
        return (1000000U / 60U);
    case som100Hz:
        return (1000000U / 100U);
    case som160Hz:
        return (1000000U / 160U);
    case som333Hz:
        return (1000000U / 333U);
    case som400Hz:
        return (1000000U / 400U);
    case som10KHzDuty:
        return (1000000U / 10000U);
    default:
        return 0;
    }
}

static void servoWrite(uint8_t ch, uint16_t us)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    if ((eServoOutputMode)chConfig->val.mode == somOnOff)
    {
        servoMgr->writeDigital(ch, us > 1500U);
    }
    else
    {
        if ((eServoOutputMode)chConfig->val.mode == som10KHzDuty)
        {
            servoMgr->writeDuty(ch, constrain(us, 1000, 2000) - 1000);
        }
        else
        {
            servoMgr->writeMicroseconds(ch, us / (chConfig->val.narrow + 1));
        }
    }
}

static bool isTorqueDollyOutput(unsigned ch)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
    const eServoOutputMode mode = (eServoOutputMode)chConfig->val.mode;
    return chConfig->val.inputChannel < 2 && mode >= som50Hz && mode <= som400Hz;
}

static uint16_t failsafeUs(unsigned ch)
{
    return config.GetPwmChannel(ch)->val.failsafe + SERVO_FAILSAFE_MIN;
}

static void writeImmediateFailsafe()
{
    for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
    {
        // Failsafe values are absolute and intentionally ignore inversion.
        if (!isTorqueDollyOutput(ch))
        {
            servoWrite(ch, failsafeUs(ch));
        }
    }
}

static void startTorqueDollyFailsafe(uint32_t now)
{
    torqueDollyRampStart = now;
    torqueDollyRamping = false;
    torqueDollyHolding = false;
    torqueDollyReleaseReady = false;

    for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
    {
        if (isTorqueDollyOutput(ch))
        {
            torqueDollyRampFrom[ch] = lastLogicalUsValid[ch] ? lastLogicalUs[ch] : failsafeUs(ch);
            torqueDollyRamping = true;
        }
    }

    // All non-motor outputs retain stock ExpressLRS immediate failsafe behavior.
    writeImmediateFailsafe();
}

static void updateTorqueDollyFailsafe(uint32_t now)
{
    if (!torqueDollyRamping)
    {
        return;
    }

    const uint32_t elapsed = now - torqueDollyRampStart;
    const uint32_t elapsedSeconds = elapsed / 1000U;
    const uint32_t elapsedMillis = elapsed % 1000U;
    const uint32_t distance = elapsedSeconds * TORQUE_DOLLY_RAMP_RATE_US_PER_SEC +
                              (elapsedMillis * TORQUE_DOLLY_RAMP_RATE_US_PER_SEC) / 1000U;
    bool reachedTarget = true;

    for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
    {
        if (!isTorqueDollyOutput(ch))
        {
            continue;
        }

        const uint16_t from = torqueDollyRampFrom[ch];
        const uint16_t target = failsafeUs(ch);
        const uint32_t delta = from < target
            ? static_cast<uint32_t>(target) - from
            : static_cast<uint32_t>(from) - target;
        const uint32_t rampedUs = distance >= delta
            ? target
            : (from < target ? static_cast<uint32_t>(from) + distance
                              : static_cast<uint32_t>(from) - distance);
        const uint16_t us = static_cast<uint16_t>(rampedUs);
        servoWrite(ch, us);
        reachedTarget = reachedTarget && us == target;
    }

    if (reachedTarget)
    {
        torqueDollyRamping = false;
        torqueDollyHolding = true;
    }
}

static bool torqueDollyCommandsAreNeutral()
{
    for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
    {
        if (!isTorqueDollyOutput(ch))
        {
            continue;
        }
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
        if (crsfVal == 0)
        {
            return false;
        }

        uint16_t commandUs = CRSF_to_US(crsfVal);
        // Compare the logical live command; failsafe itself remains absolute.
        if (chConfig->val.inverted)
        {
            commandUs = 3000U - commandUs;
        }
        const uint16_t target = failsafeUs(ch);
        const uint16_t difference = commandUs > target ? commandUs - target : target - commandUs;
        if (difference > TORQUE_DOLLY_NEUTRAL_TOLERANCE_US)
        {
            return false;
        }
    }
    return true;
}

static int servosUpdate(unsigned long now)
{
    static uint32_t lastUpdate;
    if (connectionState == wifiUpdate)
    {
        return DURATION_NEVER;
    }
    updateTorqueDollyFailsafe(now);

    if (newChannelsAvailable)
    {
        newChannelsAvailable = false;
        lastUpdate = now;
        const bool releaseThisPacket = torqueDollyHolding && torqueDollyCommandsAreNeutral();
        for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
            const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
            // crsfVal might 0 if this is a switch channel and it has not been received yet.
            if (crsfVal == 0)
            {
                continue;
            }

            // Recovery packets cannot cancel a ramp. The ramp updater is the
            // only code allowed to control motor PWM while it is active.
            if (isTorqueDollyOutput(ch) && torqueDollyRamping)
            {
                continue;
            }
            if (isTorqueDollyOutput(ch) && torqueDollyHolding)
            {
                servoWrite(ch, failsafeUs(ch));
                continue;
            }

            uint16_t us = CRSF_to_US(crsfVal);
            // Preserve normal ExpressLRS inversion behavior for live commands.
            if (chConfig->val.inverted)
            {
                us = 3000U - us;
            }
            servoWrite(ch, us);
            if (isTorqueDollyOutput(ch))
            {
                lastLogicalUs[ch] = us;
                lastLogicalUsValid[ch] = true;
            }
        }

        if (releaseThisPacket)
        {
            // The neutral packet itself remains failsafe; the next packet resumes control.
            torqueDollyHolding = false;
            torqueDollyReleaseReady = true;
        }
        else if (torqueDollyReleaseReady)
        {
            torqueDollyReleaseReady = false;
        }
    }
    else if (lastUpdate &&
             ((getLq() == 0) || (now - lastUpdate > FAILSAFE_ABS_TIMEOUT_MS)))
    {
        // A later RF loss must not re-enter from stale lastLogicalUs. Only the
        // normal live-control state is allowed to start a new ramp.
        if (torqueDollyRamping)
        {
            writeImmediateFailsafe();
        }
        else if (torqueDollyHolding)
        {
            writeImmediateFailsafe();
            for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
            {
                if (isTorqueDollyOutput(ch))
                {
                    servoWrite(ch, failsafeUs(ch));
                }
            }
        }
        else if (torqueDollyReleaseReady)
        {
            torqueDollyReleaseReady = false;
            torqueDollyHolding = true;
            writeImmediateFailsafe();
            for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
            {
                if (isTorqueDollyOutput(ch))
                {
                    servoWrite(ch, failsafeUs(ch));
                }
            }
        }
        else
        {
            startTorqueDollyFailsafe(now);
        }
        // Prevent the same loss from retriggering on every timeout.
        lastUpdate = 0;
    }

    return DURATION_IMMEDIATELY;
}

static void initialize()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return;
    }

    for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
        uint8_t pin = GPIO_PIN_PWM_OUTPUTS[ch];
#if (defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)) && (defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32))
        // Disconnect the debug UART pins if DEBUG_LOG
        if (pin == 1 || pin == 3)
        {
            pin = ServoMgr::PIN_DISCONNECTED;
        }
#endif
        // Mark servo pins that are being used for serial as disconnected
        eServoOutputMode mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;
        if (mode == somSerial)
        {
            pin = ServoMgr::PIN_DISCONNECTED;
        }
        SERVO_PINS[ch] = pin;
    }

    // Initialize all servos to low ASAP
    servoMgr = new ServoMgr(SERVO_PINS, GPIO_PIN_PWM_OUTPUTS_COUNT, 20000U);
    servoMgr->initialize();
}

static int start()
{
    for (unsigned ch = 0; servoMgr && ch < servoMgr->getOutputCnt(); ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        servoMgr->setRefreshInterval(ch, servoOutputModeToUs((eServoOutputMode)chConfig->val.mode));
    }

    return DURATION_NEVER;
}

static int event()
{
    if (servoMgr == nullptr)
    {
        return DURATION_NEVER;
    }
    if (connectionState == wifiUpdate)
    {
        servoMgr->stopAllPwm();
        torqueDollyRamping = false;
        torqueDollyHolding = false;
        torqueDollyReleaseReady = false;
        for (unsigned ch = 0; ch < servoMgr->getOutputCnt(); ++ch)
        {
            lastLogicalUsValid[ch] = false;
        }
        return DURATION_NEVER;
    }
    // A disconnected state must not freeze a Torque Dolly motor mid-ramp.
    if (torqueDollyRamping)
    {
        return DURATION_IMMEDIATELY;
    }
    if (connectionState == disconnected)
    {
        // Disconnected should come after failsafe on the RX.
        return DURATION_NEVER;
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    return servosUpdate(millis());
}

device_t ServoOut_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
};

#endif
