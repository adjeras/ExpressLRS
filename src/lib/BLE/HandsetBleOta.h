#pragma once

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)
#include <NimBLEDevice.h>

void setupHandsetBleOtaService(NimBLEService *service, NimBLEServer *server);
void processHandsetBleOta();
void setHandsetBleOtaConnected(bool connected);
void notifyHandsetBleOtaAbort();
void setHandsetBleOtaConnected(bool connected);
void notifyHandsetBleOtaDisconnected();
bool isHandsetBleOtaTransferActive();
String getHandsetRunningImageIdentity();
#endif
