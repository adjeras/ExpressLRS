#pragma once

#include "handset.h"
#include "TorqueDollyButton.h"

#if defined(TORQUE_DOLLY_RF_TEST) && defined(TORQUE_DOLLY_HANDSET) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)
struct TorqueDollyButtonFeedbackSnapshot
{
    bool active;
    bool pending;
    uint32_t blinkStartedAtMs;
};

extern volatile bool torqueDollyShutdownActive;
bool torqueDollyPowerButtonHeld();
TorqueDollyButton::Snapshot getTorqueDollyButtonSnapshot();
TorqueDollyButtonFeedbackSnapshot getTorqueDollyButtonFeedback();
#endif

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32) && defined(TARGET_TX)
enum class TorqueDollyDiagnosticState : uint8_t
{
    Locked,
    Ready,
    ControlledStop
};

struct TorqueDollyJoystickSnapshot
{
    uint32_t sequence;
    uint16_t xRaw;
    uint16_t yRaw;
    float xFiltered;
    float yFiltered;
    float xCenter;
    float yCenter;
    float xCommand;
    float yCommand;
};
#endif

class TorqueDollyHandset final : public Handset
{
public:
    void Begin() override;
    void End() override;
    void handleInput() override;
#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32) && defined(TARGET_TX)
    static TorqueDollyDiagnosticState getDiagnosticState();
    static TorqueDollyJoystickSnapshot getJoystickSnapshot();
    static uint8_t getAutoPowerOffMinutes();
    static bool setAutoPowerOffMinutes(uint8_t minutes);
    static bool consumeAutoPowerOffRequest();
    static bool isAutoPowerOffSuspended();
#endif
    bool IsArmed() override { return false; }
};
