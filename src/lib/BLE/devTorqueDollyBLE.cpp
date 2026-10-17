#include "targets.h"

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)

#if !defined(TORQUE_DOLLY_HANDSET)
#error "TORQUE_DOLLY_BLE_TELEMETRY requires TORQUE_DOLLY_HANDSET"
#endif

#include "devTorqueDollyBLE.h"
#include "logging.h"
#include "TorqueDollyHandset.h"

#include <NimBLEDevice.h>
#include <cstdio>

namespace {

constexpr char DEVICE_NAME[] = "TorqueDolly-TX";
constexpr char SERVICE_UUID[] = "7f2a0001-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char CHARACTERISTIC_UUID[] = "7f2a0002-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char STATUS_CHARACTERISTIC_UUID[] = "7f2a0003-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char IDENTIFICATION[] = "TD-TX-BLE-B1";
constexpr size_t STATUS_VALUE_MAX_LENGTH = 64;
constexpr uint16_t ADVERTISING_INTERVAL_UNITS = 800; // 800 * 0.625 ms = 500 ms

static bool started = false;

class StatusCharacteristicCallbacks final : public NimBLECharacteristicCallbacks
{
public:
    void onRead(NimBLECharacteristic *characteristic) override
    {
        const TorqueDollyDiagnosticState state =
            TorqueDollyHandset::getDiagnosticState();
        const char *stateName = "LOCKED";
        if (state == TorqueDollyDiagnosticState::Ready)
        {
            stateName = "READY";
        }
        else if (state == TorqueDollyDiagnosticState::ControlledStop)
        {
            stateName = "CONTROLLED_STOP";
        }

        char value[STATUS_VALUE_MAX_LENGTH];
        const int length = snprintf(
            value,
            sizeof(value),
            "state=%s;uptime_ms=%lu",
            stateName,
            static_cast<unsigned long>(millis())
        );
        if (length < 0 || static_cast<size_t>(length) >= sizeof(value))
        {
            Serial.println("Torque Dolly BLE: status formatting failed");
            return;
        }

        characteristic->setValue(
            reinterpret_cast<const uint8_t *>(value),
            static_cast<size_t>(length));
    }
};

static StatusCharacteristicCallbacks statusCallbacks;

static int start()
{
    if (started)
        return DURATION_NEVER;

    started = true;
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);

    NimBLEServer *server = NimBLEDevice::createServer();
    if (server == nullptr)
    {
        DBGLN("Torque Dolly BLE: server creation failed");
        return DURATION_NEVER;
    }

    NimBLEService *service = server->createService(SERVICE_UUID);
    if (service == nullptr)
    {
        DBGLN("Torque Dolly BLE: service creation failed");
        return DURATION_NEVER;
    }

    NimBLECharacteristic *characteristic = service->createCharacteristic(
        CHARACTERISTIC_UUID, NIMBLE_PROPERTY::READ, sizeof(IDENTIFICATION) - 1);
    if (characteristic == nullptr)
    {
        DBGLN("Torque Dolly BLE: characteristic creation failed");
        return DURATION_NEVER;
    }

    characteristic->setValue(
        reinterpret_cast<const uint8_t *>(IDENTIFICATION),
        sizeof(IDENTIFICATION) - 1);
    if (characteristic->getDataLength() != sizeof(IDENTIFICATION) - 1)
    {
        Serial.println("Torque Dolly BLE: characteristic value length invalid");
        return DURATION_NEVER;
    }

    NimBLECharacteristic *statusCharacteristic = service->createCharacteristic(
        STATUS_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ,
        STATUS_VALUE_MAX_LENGTH);
    if (statusCharacteristic == nullptr)
    {
        DBGLN("Torque Dolly BLE: status characteristic creation failed");
        return DURATION_NEVER;
    }

    statusCharacteristic->setValue(
        reinterpret_cast<const uint8_t *>("state=LOCKED;uptime_ms=0"),
        sizeof("state=LOCKED;uptime_ms=0") - 1);
    statusCharacteristic->setCallbacks(&statusCallbacks);

    if (!service->start())
    {
        DBGLN("Torque Dolly BLE: service start failed");
        return DURATION_NEVER;
    }

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    if (advertising == nullptr)
    {
        DBGLN("Torque Dolly BLE: advertising unavailable");
        return DURATION_NEVER;
    }

    advertising->addServiceUUID(service->getUUID());
    advertising->setName(DEVICE_NAME);
    advertising->setMinInterval(ADVERTISING_INTERVAL_UNITS);
    advertising->setMaxInterval(ADVERTISING_INTERVAL_UNITS);
    advertising->setScanResponse(true);
    server->advertiseOnDisconnect(true);
    if (!advertising->start())
    {
        DBGLN("Torque Dolly BLE: advertising start failed");
        return DURATION_NEVER;
    }

    DBGLN("Torque Dolly BLE: advertising started on core %d", xPortGetCoreID());
    return DURATION_NEVER;
}

} // namespace

device_t TorqueDollyBLE_device = {
    .initialize = nullptr,
    .start = start,
    .event = nullptr,
    .timeout = nullptr,
};

#endif
