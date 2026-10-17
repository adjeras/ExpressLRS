#pragma once

#include "handset.h"

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32) && defined(TARGET_TX)
enum class TorqueDollyDiagnosticState : uint8_t
{
    Locked,
    Ready,
    ControlledStop
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
#endif
    bool IsArmed() override { return false; }
};
