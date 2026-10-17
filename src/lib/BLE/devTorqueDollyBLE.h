#pragma once

#include "device.h"

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)

#if defined(USE_BLE_JOYSTICK)
#error "TORQUE_DOLLY_BLE_TELEMETRY is incompatible with USE_BLE_JOYSTICK"
#endif

extern device_t TorqueDollyBLE_device;

#endif
