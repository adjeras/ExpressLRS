#include "targets.h"

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)

#if !defined(TORQUE_DOLLY_HANDSET)
#error "TORQUE_DOLLY_BLE_TELEMETRY requires TORQUE_DOLLY_HANDSET"
#endif

#include "devTorqueDollyBLE.h"
#include "logging.h"
#include "TorqueDollyHandset.h"
#include "TorqueDollyControlParameters.h"

#include <NimBLEDevice.h>
#include <cstdio>

namespace {

constexpr char DEVICE_NAME[] = "TorqueDolly-TX";
constexpr char SERVICE_UUID[] = "7f2a0001-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char CHARACTERISTIC_UUID[] = "7f2a0002-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char STATUS_CHARACTERISTIC_UUID[] = "7f2a0003-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char JOYSTICK_DIAGNOSTIC_CHARACTERISTIC_UUID[] = "7f2a0004-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char AUTO_POWER_OFF_CHARACTERISTIC_UUID[] = "7f2a0005-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char IDENTIFICATION[] = "TD-TX-BLE-B1";
constexpr size_t STATUS_VALUE_MAX_LENGTH = 64;
constexpr size_t JOYSTICK_DIAGNOSTIC_VALUE_LENGTH = 20;
constexpr uint16_t ADVERTISING_INTERVAL_UNITS = 800; // 800 * 0.625 ms = 500 ms

// Joystick diagnostic payload: exactly 20 little-endian bytes.
// 0: uint32 sequence; 4/6: uint16 raw X/Y ADC counts.
// 8/10/12/14: uint16 filtered X/Y and calibration centers, decoded / 16.0.
// 16/18: signed int16 Q8.8 calculated X/Y commands, decoded / 256.0.
// The command fields are safety-gated and therefore zero while LOCKED;
// raw, filtered, and center fields remain available for drift diagnosis.

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

class JoystickDiagnosticCharacteristicCallbacks final : public NimBLECharacteristicCallbacks
{
public:
    void onRead(NimBLECharacteristic *characteristic) override
    {
        const TorqueDollyJoystickSnapshot snapshot =
            TorqueDollyHandset::getJoystickSnapshot();
        uint8_t value[JOYSTICK_DIAGNOSTIC_VALUE_LENGTH];

        auto writeU16 = [&value](size_t offset, uint16_t number) {
            value[offset] = static_cast<uint8_t>(number & 0xff);
            value[offset + 1] = static_cast<uint8_t>(number >> 8);
        };
        auto writeU32 = [&value](size_t offset, uint32_t number) {
            value[offset] = static_cast<uint8_t>(number & 0xff);
            value[offset + 1] = static_cast<uint8_t>((number >> 8) & 0xff);
            value[offset + 2] = static_cast<uint8_t>((number >> 16) & 0xff);
            value[offset + 3] = static_cast<uint8_t>(number >> 24);
        };
        auto toQ8_8 = [](float number) {
            return static_cast<int16_t>(constrain(
                lroundf(number * 256.0f), -32768L, 32767L));
        };
        auto toAdcQ4_12 = [](float number) {
            return static_cast<uint16_t>(constrain(
                lroundf(number * 16.0f), 0L, 65535L));
        };
        auto writeQ8_8 = [&value, &toQ8_8](size_t offset, float number) {
            const uint16_t encoded = static_cast<uint16_t>(toQ8_8(number));
            value[offset] = static_cast<uint8_t>(encoded & 0xff);
            value[offset + 1] = static_cast<uint8_t>(encoded >> 8);
        };
        auto writeAdcQ4_12 = [&value, &toAdcQ4_12](size_t offset, float number) {
            const uint16_t encoded = toAdcQ4_12(number);
            value[offset] = static_cast<uint8_t>(encoded & 0xff);
            value[offset + 1] = static_cast<uint8_t>(encoded >> 8);
        };

        writeU32(0, snapshot.sequence);
        writeU16(4, snapshot.xRaw);
        writeU16(6, snapshot.yRaw);
        writeAdcQ4_12(8, snapshot.xFiltered);
        writeAdcQ4_12(10, snapshot.yFiltered);
        writeAdcQ4_12(12, snapshot.xCenter);
        writeAdcQ4_12(14, snapshot.yCenter);
        writeQ8_8(16, snapshot.xCommand);
        writeQ8_8(18, snapshot.yCommand);

        characteristic->setValue(value, sizeof(value));
    }
};

static JoystickDiagnosticCharacteristicCallbacks joystickDiagnosticCallbacks;

class AutoPowerOffCharacteristicCallbacks final : public NimBLECharacteristicCallbacks
{
public:
    void onRead(NimBLECharacteristic *characteristic) override
    {
        char value[4];
        const int length = snprintf(
            value, sizeof(value), "%u",
            TorqueDollyHandset::getAutoPowerOffMinutes());
        characteristic->setValue(reinterpret_cast<const uint8_t *>(value), length);
    }

    void onWrite(NimBLECharacteristic *characteristic) override
    {
        const std::string raw = characteristic->getValue();
        char *end = nullptr;
        const long minutes = strtol(raw.c_str(), &end, 10);
        const bool valid = end != raw.c_str() && *end == '\\0' &&
            minutes >= TorqueDollyControlParameters::AUTO_POWER_OFF_MINUTES_MIN &&
            minutes <= TorqueDollyControlParameters::AUTO_POWER_OFF_MINUTES_MAX;
        if (!valid || !TorqueDollyHandset::setAutoPowerOffMinutes(
            static_cast<uint8_t>(minutes)))
        {
            characteristic->setValue("ERR");
            return;
        }
        characteristic->setValue(std::to_string(
            TorqueDollyHandset::getAutoPowerOffMinutes()));
    }
};

static AutoPowerOffCharacteristicCallbacks autoPowerOffCallbacks;

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

    NimBLECharacteristic *joystickDiagnosticCharacteristic =
        service->createCharacteristic(
            JOYSTICK_DIAGNOSTIC_CHARACTERISTIC_UUID,
            NIMBLE_PROPERTY::READ,
            JOYSTICK_DIAGNOSTIC_VALUE_LENGTH);
    if (joystickDiagnosticCharacteristic == nullptr)
    {
        DBGLN("Torque Dolly BLE: joystick diagnostic characteristic creation failed");
        return DURATION_NEVER;
    }

    uint8_t initialJoystickDiagnostic[JOYSTICK_DIAGNOSTIC_VALUE_LENGTH] = {};
    joystickDiagnosticCharacteristic->setValue(
        initialJoystickDiagnostic,
        sizeof(initialJoystickDiagnostic));
    joystickDiagnosticCharacteristic->setCallbacks(
        &joystickDiagnosticCallbacks);

    NimBLECharacteristic *autoPowerOffCharacteristic = service->createCharacteristic(
        AUTO_POWER_OFF_CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE,
        4);
    if (autoPowerOffCharacteristic == nullptr)
    {
        DBGLN("Torque Dolly BLE: auto power-off characteristic creation failed");
        return DURATION_NEVER;
    }
    autoPowerOffCharacteristic->setCallbacks(&autoPowerOffCallbacks);
    autoPowerOffCharacteristic->setValue(std::to_string(
        TorqueDollyHandset::getAutoPowerOffMinutes()));

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
