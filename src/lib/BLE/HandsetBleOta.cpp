#include "targets.h"

#if defined(TORQUE_DOLLY_BLE_TELEMETRY) && defined(PLATFORM_ESP32_S3) && defined(TARGET_TX)

#include "HandsetBleOta.h"
#include "HandsetOtaImagePolicy.h"
#include "TorqueDollyHandset.h"
#include "options.h"
#include "device.h"
#include <Update.h>
#include <Preferences.h>
#include <esp_app_format.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <mbedtls/sha256.h>
#include <cstring>
#include <string>

namespace {
constexpr char kCapabilityUuid[] = "7f2a0008-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char kStatusUuid[] = "7f2a0009-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char kControlUuid[] = "7f2a000a-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char kDataUuid[] = "7f2a000b-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char kIdentityUuid[] = "7f2a000c-6d4e-4c8a-9b11-2e4d7a8f3101";
constexpr char kPreferencesNamespace[] = "td_ble_ota";
constexpr char kImageSizeKey[] = "image_size";
constexpr char kImageIdentityKey[] = "image_key";
constexpr char kImageSha256Key[] = "image_sha256";
constexpr uint32_t kImageIdentityTag = 0x54444F54; // TDOT
constexpr size_t kSha256HexLength = 64;
constexpr size_t kHashHexLength = kSha256HexLength;
constexpr size_t kChunkBytes = 180;
constexpr size_t kPacketQueueDepth = 4;
constexpr size_t kControlQueueDepth = 4;
constexpr size_t kPreflightBytes = 16 * 1024;
constexpr size_t kMaxTargetMarkerBytes = 128;
constexpr size_t kPreflightBufferBytes = kPreflightBytes + kMaxTargetMarkerBytes - 1;
constexpr uint32_t kTransferTimeoutMs = 5000;
constexpr uint32_t kMaxHandsetImageBytes = 1966080;
constexpr uint32_t kMaximumSlotBytes = kMaxHandsetImageBytes;

struct Packet {
    uint32_t offset = 0;
    uint16_t length = 0;
    uint8_t bytes[kChunkBytes] = {};
};

NimBLECharacteristic *gStatus = nullptr;
NimBLECharacteristic *gIdentity = nullptr;
portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool gConnected = false;
volatile bool gCancelRequested = false;
volatile bool gSessionActive = false;
volatile bool gCommitRequested = false;
volatile bool gFinalizing = false;
bool gWriterActive = false;
bool gUpdateFinalized = false;
bool gShaActive = false;
bool gTargetConfirmed = false;
uint32_t gDeclaredSize = 0;
uint32_t gAcceptedOffset = 0;
uint32_t gWrittenOffset = 0;
uint32_t gLastActivityMs = 0;
char gExpectedSha256[kSha256HexLength + 1] = {};
uint8_t gPreflight[kPreflightBufferBytes] = {};
size_t gPreflightLength = 0;
Packet gPackets[kPacketQueueDepth];
size_t gPacketHead = 0;
size_t gPacketTail = 0;
size_t gPacketCount = 0;
mbedtls_sha256_context gSha;
const esp_partition_t *gRunningPartition = nullptr;
const esp_partition_t *gInactivePartition = nullptr;
uint32_t gPreviousImageSize = 0;
uint32_t gPreviousIdentityTag = 0;
String gPreviousImageSha256;
bool gHadPreviousIdentity = false;

struct Command {
    enum Type : uint8_t { Begin } type = Begin;
    uint32_t size = 0;
    char hash[kSha256HexLength + 1] = {};
};
Command gCommands[kControlQueueDepth];
size_t gCommandHead = 0;
size_t gCommandTail = 0;
size_t gCommandCount = 0;

size_t handsetTargetMarkerLength() {
    return target_name_size;
}

void setTextValue(NimBLECharacteristic *characteristic, const char *text, size_t length) {
    if (characteristic == nullptr || text == nullptr) return;
    characteristic->setValue(reinterpret_cast<const uint8_t *>(text), length);
}

void setTextValue(NimBLECharacteristic *characteristic, const String &text) {
    setTextValue(characteristic, text.c_str(), text.length());
}

void publishStatus(const String &status) {
    if (gStatus == nullptr) return;
    setTextValue(gStatus, status);
    if (gConnected) gStatus->notify();
}

void recordOtaActivity() {
    portENTER_CRITICAL(&gMux);
    gLastActivityMs = millis();
    portEXIT_CRITICAL(&gMux);
}

void clearPacketQueue() {
    portENTER_CRITICAL(&gMux);
    gPacketHead = gPacketTail = gPacketCount = 0;
    gAcceptedOffset = 0;
    portEXIT_CRITICAL(&gMux);
}

void restoreIdentityMetadata() {
    Preferences preferences;
    if (!preferences.begin(kPreferencesNamespace, false)) return;
    if (gHadPreviousIdentity) {
        preferences.putUInt(kImageSizeKey, gPreviousImageSize);
        preferences.putUInt(kImageIdentityKey, gPreviousIdentityTag);
        preferences.putString(kImageSha256Key, gPreviousImageSha256);
    } else {
        preferences.remove(kImageSizeKey);
        preferences.remove(kImageIdentityKey);
        preferences.remove(kImageSha256Key);
    }
    preferences.end();
}

void releaseMaintenanceSession() {
    portENTER_CRITICAL(&gMux);
    gSessionActive = false;
    gCommitRequested = false;
    gCancelRequested = false;
    gFinalizing = false;
    portEXIT_CRITICAL(&gMux);
    TorqueDollyHandset::endOtaTransfer();
}

void abortTransfer(const char *reason, const char *cause = nullptr) {
    uint32_t nowMs;
    uint32_t lastActivityMs;
    bool connected;
    portENTER_CRITICAL(&gMux);
    nowMs = millis();
    lastActivityMs = gLastActivityMs;
    connected = gConnected;
    portEXIT_CRITICAL(&gMux);
    Serial.printf("HandsetOtaPerf abort cause=%s connected=%u now_ms=%lu last_activity_ms=%lu idle_ms=%lu timeout_ms=%lu\n",
        cause != nullptr ? cause : reason, connected ? 1U : 0U, static_cast<unsigned long>(nowMs),
        static_cast<unsigned long>(lastActivityMs),
        static_cast<unsigned long>(nowMs - lastActivityMs),
        static_cast<unsigned long>(kTransferTimeoutMs));
    if (gWriterActive && !gUpdateFinalized) Update.abort();
    if (gUpdateFinalized && gRunningPartition != nullptr) {
        // Update.end(false) selects a candidate. Until reboot, cancellation
        // restores the old running slot and the old running-image identity.
        esp_ota_set_boot_partition(gRunningPartition);
        restoreIdentityMetadata();
    }
    if (gShaActive) {
        mbedtls_sha256_free(&gSha);
        gShaActive = false;
    }
    gWriterActive = false;
    gUpdateFinalized = false;
    gTargetConfirmed = false;
    gPreflightLength = 0;
    gExpectedSha256[0] = '\0';
    clearPacketQueue();
    releaseMaintenanceSession();
    publishStatus(String("ABORTED;reason=") + reason + ";maintenance=locked");
}

bool parseBegin(const std::string &value, uint32_t &size, char *hash) {
    constexpr char kBegin[] = "BEGIN;size=";
    constexpr char kHash[] = ";sha256=";
    if (value.compare(0, sizeof(kBegin) - 1, kBegin) != 0) return false;
    const size_t separator = value.find(kHash, sizeof(kBegin) - 1);
    if (separator == std::string::npos || separator == sizeof(kBegin) - 1) return false;
    const size_t digits = separator - (sizeof(kBegin) - 1);
    if (digits > 7) return false;
    uint32_t parsedSize = 0;
    for (size_t i = sizeof(kBegin) - 1; i < separator; ++i) {
        if (value[i] < '0' || value[i] > '9') return false;
        parsedSize = parsedSize * 10 + static_cast<uint32_t>(value[i] - '0');
        if (parsedSize > kMaxHandsetImageBytes) return false;
    }
    const size_t hashStart = separator + sizeof(kHash) - 1;
    if (parsedSize < 28 || value.size() - hashStart != kSha256HexLength) return false;
    for (size_t i = 0; i < kSha256HexLength; ++i) {
        const char c = value[hashStart + i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
        hash[i] = c;
    }
    hash[kSha256HexLength] = '\0';
    size = parsedSize;
    return true;
}

bool loadCompiledTargetMarker(uint8_t *marker, size_t &markerSize) {
    // TARGET_NAME expands to the exact PIO environment, while TARGET_NAME's
    // compiled options.cpp object prefixes it with the four-byte magic.
    if (target_name_size < 6 || target_name_size > 128) return false;
    markerSize = target_name_size;
    memcpy(marker, target_name, markerSize);
    static constexpr uint8_t kMagic[] = {0xBE, 0xEF, 0xCA, 0xFE};
    return marker[0] == kMagic[0] && marker[1] == kMagic[1] &&
        marker[2] == kMagic[2] && marker[3] == kMagic[3] &&
        marker[markerSize - 1] == 0;
}

bool imageContainsHandsetTargetMarker(size_t searchBytes) {
    uint8_t marker[kMaxTargetMarkerBytes] = {};
    size_t markerSize = 0;
    if (!loadCompiledTargetMarker(marker, markerSize) || gPreflightLength < markerSize) return false;
    const size_t possibleStarts = gPreflightLength - markerSize + 1;
    const size_t startsToScan = min(possibleStarts, searchBytes);
    for (size_t offset = 0; offset < startsToScan; ++offset) {
        if (memcmp(gPreflight + offset, marker, markerSize) == 0) return true;
    }
    return false;
}

bool startInactivePartitionWriter() {
    gRunningPartition = esp_ota_get_running_partition();
    gInactivePartition = gRunningPartition == nullptr ? nullptr :
        esp_ota_get_next_update_partition(gRunningPartition);
    const esp_partition_t *defaultUpdate = esp_ota_get_next_update_partition(nullptr);
    if (gRunningPartition == nullptr || gInactivePartition == nullptr || defaultUpdate == nullptr ||
        gInactivePartition->address == gRunningPartition->address ||
        gInactivePartition->address != defaultUpdate->address ||
        gInactivePartition->type != ESP_PARTITION_TYPE_APP ||
        gInactivePartition->subtype == ESP_PARTITION_SUBTYPE_APP_FACTORY ||
        gDeclaredSize > gInactivePartition->size || gInactivePartition->size > kMaxHandsetImageBytes) {
        publishStatus("ERROR;inactive-partition-or-size");
        return false;
    }
    if (!Update.begin(gDeclaredSize, U_FLASH)) {
        publishStatus(String("ERROR;Update.begin;") + Update.errorString());
        return false;
    }
    gWriterActive = true;
    mbedtls_sha256_init(&gSha);
    mbedtls_sha256_starts(&gSha, 0);
    gShaActive = true;
    if (Update.write(gPreflight, gPreflightLength) != gPreflightLength) {
        publishStatus("ERROR;first-flash-write");
        return false;
    }
    mbedtls_sha256_update(&gSha, gPreflight, gPreflightLength);
    gWrittenOffset = static_cast<uint32_t>(gPreflightLength);
    gTargetConfirmed = true;
    recordOtaActivity();
    publishStatus(String("TARGET_OK;target=") + reinterpret_cast<const char *>(target_name + 4));
    return true;
}

bool popPacket(Packet &packet) {
    bool found = false;
    portENTER_CRITICAL(&gMux);
    if (gPacketCount != 0) {
        packet = gPackets[gPacketTail];
        gPacketTail = (gPacketTail + 1) % kPacketQueueDepth;
        --gPacketCount;
        found = true;
    }
    portEXIT_CRITICAL(&gMux);
    return found;
}

class HandsetOtaServerCallbacks final : public NimBLEServerCallbacks {
public:
    void onConnect(NimBLEServer *) override { setHandsetBleOtaConnected(true); }
    void onDisconnect(NimBLEServer *) override { notifyHandsetBleOtaDisconnected(); }
};
static HandsetOtaServerCallbacks gServerCallbacks;

class IdentityCallbacks final : public NimBLECharacteristicCallbacks {
public:
    void onRead(NimBLECharacteristic *characteristic) override {
        setTextValue(characteristic, getHandsetRunningImageIdentity());
    }
};
static IdentityCallbacks gIdentityCallbacks;

class ControlCallbacks final : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *characteristic) override {
        const std::string value = characteristic->getValue();
        if (value == "ABORT") {
            notifyHandsetBleOtaAbort();
            return;
        }
        if (value == "EXIT") {
            const bool exited = TorqueDollyHandset::exitMaintenance();
            publishStatus(exited ? "LOCKED" : "ERROR;transfer-active;maintenance=locked");
            return;
        }
        if (value == "COMMIT") {
            bool valid = false;
            portENTER_CRITICAL(&gMux);
            valid = gSessionActive && gTargetConfirmed && gWriterActive && gConnected && !gCancelRequested;
            if (valid) gCommitRequested = true;
            portEXIT_CRITICAL(&gMux);
            publishStatus(valid ? "COMMIT_QUEUED" : "ERROR;inactive-or-preflight");
            return;
        }
        uint32_t size = 0;
        char sha256[kSha256HexLength + 1] = {};
        if (!parseBegin(value, size, sha256)) {
            publishStatus("ERROR;command");
            return;
        }
        if (!gConnected || !TorqueDollyHandset::beginOtaTransfer()) {
            publishStatus("ERROR;maintenance-required-or-transfer-active");
            return;
        }
        bool accepted = false;
        portENTER_CRITICAL(&gMux);
        if (!gSessionActive && gCommandCount == 0 && !gFinalizing && gConnected) {
            gSessionActive = true;
            gCancelRequested = false;
            gCommitRequested = false;
            gWriterActive = false;
            gDeclaredSize = size;
            gAcceptedOffset = gWrittenOffset = 0;
            gPacketHead = gPacketTail = gPacketCount = 0;
            gCommandHead = gCommandTail = gCommandCount = 0;
            gPreflightLength = 0;
            gTargetConfirmed = false;
            gUpdateFinalized = false;
            gHadPreviousIdentity = false;
            gPreviousImageSize = 0;
            gPreviousIdentityTag = 0;
            gPreviousImageSha256 = "";
            memcpy(gExpectedSha256, sha256, sizeof(gExpectedSha256));
            accepted = true;
        }
        portEXIT_CRITICAL(&gMux);
        if (!accepted) {
            TorqueDollyHandset::endOtaTransfer();
            publishStatus("ERROR;busy");
            return;
        }
        Preferences prefs;
        if (prefs.begin(kPreferencesNamespace, true)) {
            gHadPreviousIdentity = prefs.isKey(kImageSizeKey);
            gPreviousImageSize = prefs.getUInt(kImageSizeKey, 0);
            gPreviousIdentityTag = prefs.getUInt(kImageIdentityKey, 0);
            gPreviousImageSha256 = prefs.getString(kImageSha256Key, "");
            prefs.end();
        }
        Command command;
        command.type = Command::Begin;
        command.size = size;
        memcpy(command.hash, sha256, sizeof(command.hash));
        portENTER_CRITICAL(&gMux);
        gCommands[gCommandHead] = command;
        gCommandHead = (gCommandHead + 1) % kControlQueueDepth;
        ++gCommandCount;
        gLastActivityMs = millis();
        portEXIT_CRITICAL(&gMux);
        // Wake the existing device scheduler so its event callback can switch
        // this BLE device to the OTA-only cadence without moving OTA work here.
        devicesTriggerEvent();
        publishStatus(String("READY;offset=0;size=") + size + ";slot=inactive;preflight=target-required");
    }
};
static ControlCallbacks gControlCallbacks;

class DataCallbacks final : public NimBLECharacteristicCallbacks {
public:
    void onWrite(NimBLECharacteristic *characteristic) override {
        const std::string raw = characteristic->getValue();
        if (raw.size() < 5 || raw.size() - 4 > kChunkBytes) {
            notifyHandsetBleOtaAbort();
            publishStatus("ERROR;chunk-size");
            return;
        }
        const uint32_t offset = static_cast<uint8_t>(raw[0]) |
            (static_cast<uint32_t>(static_cast<uint8_t>(raw[1])) << 8) |
            (static_cast<uint32_t>(static_cast<uint8_t>(raw[2])) << 16) |
            (static_cast<uint32_t>(static_cast<uint8_t>(raw[3])) << 24);
        const size_t length = raw.size() - 4;
        bool accepted = false;
        portENTER_CRITICAL(&gMux);
        if (gSessionActive && gConnected && !gCancelRequested && !gFinalizing &&
            offset == gAcceptedOffset && offset <= gDeclaredSize && length <= gDeclaredSize - offset &&
            gPacketCount < kPacketQueueDepth) {
            Packet &packet = gPackets[gPacketHead];
            packet.offset = offset;
            packet.length = static_cast<uint16_t>(length);
            memcpy(packet.bytes, raw.data() + 4, length);
            gPacketHead = (gPacketHead + 1) % kPacketQueueDepth;
            ++gPacketCount;
            gAcceptedOffset += static_cast<uint32_t>(length);
            gLastActivityMs = millis();
            accepted = true;
        } else if (gSessionActive && !gFinalizing) {
            gCancelRequested = true;
        }
        portEXIT_CRITICAL(&gMux);
        if (!accepted) publishStatus("ERROR;offset-or-busy");
    }
};
static DataCallbacks gDataCallbacks;
}

void setupHandsetBleOtaService(NimBLEService *service, NimBLEServer *server) {
    if (!service || !server) return;
    server->setCallbacks(&gServerCallbacks, false);
    gStatus = service->createCharacteristic(kStatusUuid, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 128);
    setTextValue(gStatus, "IDLE;maintenance-required=1", sizeof("IDLE;maintenance-required=1") - 1);
    char capability[160];
    const int capabilityLength = snprintf(capability, sizeof(capability),
        "proto=1;target=%s;app-only=1;sha256=required;signature=none;rollback=unverified;size=%lu",
        reinterpret_cast<const char *>(target_name + 4), static_cast<unsigned long>(kMaximumSlotBytes));
    auto *cap = service->createCharacteristic(kCapabilityUuid, NIMBLE_PROPERTY::READ, sizeof(capability));
    if (capabilityLength > 0 && static_cast<size_t>(capabilityLength) < sizeof(capability)) {
        setTextValue(cap, capability, static_cast<size_t>(capabilityLength));
    }
    gIdentity = service->createCharacteristic(kIdentityUuid, NIMBLE_PROPERTY::READ, 65);
    gIdentity->setCallbacks(&gIdentityCallbacks);
    setTextValue(gIdentity, getHandsetRunningImageIdentity());
    auto *control = service->createCharacteristic(kControlUuid, NIMBLE_PROPERTY::WRITE, 128);
    control->setCallbacks(&gControlCallbacks);
    auto *data = service->createCharacteristic(kDataUuid, NIMBLE_PROPERTY::WRITE, 184);
    data->setCallbacks(&gDataCallbacks);
}

void setHandsetBleOtaConnected(bool value) {
    portENTER_CRITICAL(&gMux);
    gConnected = value;
    if (!value && gSessionActive && !gFinalizing) {
        gCancelRequested = true;
        gCommandHead = gCommandTail = gCommandCount = 0;
        gPacketHead = gPacketTail = gPacketCount = 0;
    }
    portEXIT_CRITICAL(&gMux);
}

void notifyHandsetBleOtaAbort() {
    portENTER_CRITICAL(&gMux);
    if (gSessionActive && !gFinalizing) {
        gCancelRequested = true;
        gCommandHead = gCommandTail = gCommandCount = 0;
        gPacketHead = gPacketTail = gPacketCount = 0;
    }
    portEXIT_CRITICAL(&gMux);
}

void notifyHandsetBleOtaDisconnected() { setHandsetBleOtaConnected(false); }
bool isHandsetBleOtaTransferActive() { return gSessionActive; }

String getHandsetRunningImageIdentity() {
    Preferences prefs;
    if (!prefs.begin(kPreferencesNamespace, true)) return String();
    const uint32_t imageSize = prefs.getUInt(kImageSizeKey, 0);
    const uint32_t identityTag = prefs.getUInt(kImageIdentityKey, 0);
    const String expected = prefs.getString(kImageSha256Key, "");
    prefs.end();
    if (identityTag != kImageIdentityTag || imageSize < 28 || imageSize > kMaximumSlotBytes || expected.length() != kHashHexLength) return String();
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running || imageSize > running->size) return String();
    esp_image_header_t header = {};
    if (esp_partition_read(running, 0, &header, sizeof(header)) != ESP_OK || header.magic != ESP_IMAGE_HEADER_MAGIC ||
        header.segment_count == 0 || header.segment_count > 16) return String();
    uint8_t buffer[1024];
    mbedtls_sha256_context hash;
    mbedtls_sha256_init(&hash);
    mbedtls_sha256_starts(&hash, 0);
    for (size_t offset = 0; offset < imageSize; offset += sizeof(buffer)) {
        const size_t count = min(sizeof(buffer), imageSize - offset);
        if (esp_partition_read(running, offset, buffer, count) != ESP_OK) {
            mbedtls_sha256_free(&hash);
            return String();
        }
        mbedtls_sha256_update(&hash, buffer, count);
    }
    uint8_t digest[32];
    mbedtls_sha256_finish(&hash, digest);
    mbedtls_sha256_free(&hash);
    char actual[65] = {};
    for (size_t i = 0; i < sizeof(digest); ++i) snprintf(&actual[i * 2], 3, "%02x", digest[i]);
    return String(actual).equalsIgnoreCase(expected) ? String(actual) : String();
}

void processHandsetBleOta() {
    bool cancel = false;
    bool connected = true;
    bool maintenanceActive = true;
    bool timedOut = false;
    const bool sessionActive = gSessionActive;
    if (sessionActive) maintenanceActive = TorqueDollyHandset::isMaintenanceActive();

    uint32_t nowMs = 0;
    uint32_t lastActivityMs = 0;
    portENTER_CRITICAL(&gMux);
    connected = gConnected;
    if (gSessionActive) {
        // Sample time only after acquiring the same lock used by BLE writes
        // to update gLastActivityMs. This prevents a stale-now/new-timestamp
        // unsigned underflow from appearing as a huge idle interval.
        nowMs = millis();
        lastActivityMs = gLastActivityMs;
        timedOut = static_cast<uint32_t>(nowMs - lastActivityMs) > kTransferTimeoutMs;
        if (!connected || !maintenanceActive || timedOut) gCancelRequested = true;
    }
    cancel = gCancelRequested;
    if (cancel) {
        gCancelRequested = false;
    } else if (gCommandCount != 0 && !gFinalizing) {
        gCommandTail = (gCommandTail + 1) % kControlQueueDepth;
        --gCommandCount;
    }
    portEXIT_CRITICAL(&gMux);
    if (cancel) {
        const char *reason = !connected ? "disconnect" :
            !maintenanceActive ? "maintenance-exit" :
            timedOut ? "inactivity-timeout" : "abort";
        abortTransfer(reason);
        return;
    }
    if (!sessionActive) return;

    Packet packet;
    if (popPacket(packet)) {
        if (!gWriterActive) {
            const size_t markerLength = handsetTargetMarkerLength();
            const size_t searchBytes = min(static_cast<size_t>(gDeclaredSize), kPreflightBytes);
            if (markerLength == 0 || markerLength > kMaxTargetMarkerBytes ||
                packet.offset != gPreflightLength || gPreflightLength >= kPreflightBufferBytes) {
                publishStatus("ERROR;preflight-size-or-offset"); abortTransfer("preflight"); return;
            }
            // Search marker starts within the first min(image size, 16 KiB),
            // while retaining enough lookahead to match one crossing that boundary.
            const size_t preflightLimit = min(static_cast<size_t>(gDeclaredSize),
                searchBytes + markerLength - 1);
            const size_t appendLength = min(static_cast<size_t>(packet.length),
                preflightLimit - gPreflightLength);
            memcpy(gPreflight + gPreflightLength, packet.bytes, appendLength);
            gPreflightLength += appendLength;
            if (gPreflightLength >= 24 && !HandsetOtaImagePolicy::validEsp32S3Header(gPreflight, gPreflightLength)) {
                publishStatus("ERROR;image-header"); abortTransfer("image-header"); return;
            }

            const bool targetFound = imageContainsHandsetTargetMarker(searchBytes);
            if (!targetFound) {
                if (gPreflightLength >= preflightLimit || gPreflightLength >= gDeclaredSize) {
                    publishStatus("ERROR;target-marker-not-found");
                    abortTransfer("target-marker-not-found");
                    return;
                }
                recordOtaActivity();
                publishStatus(String("ACK;offset=") + gAcceptedOffset + ";preflight=1");
                return;
            }
            if (!startInactivePartitionWriter()) {
                publishStatus("ERROR;writer-begin-or-first-write"); abortTransfer("writer-start"); return;
            }
            const size_t remainderLength = packet.length - appendLength;
            if (remainderLength != 0) {
                uint8_t *remainder = packet.bytes + appendLength;
                if (Update.write(remainder, remainderLength) != remainderLength) {
                    publishStatus("ERROR;flash-write"); abortTransfer("flash-write"); return;
                }
                mbedtls_sha256_update(&gSha, remainder, remainderLength);
                gWrittenOffset += static_cast<uint32_t>(remainderLength);
                recordOtaActivity();
            }
            publishStatus(String("ACK;offset=") + gWrittenOffset + ";target=confirmed");
        } else {
            if (packet.offset != gWrittenOffset || Update.write(packet.bytes, packet.length) != packet.length) {
                publishStatus("ERROR;flash-write"); abortTransfer("flash-write"); return;
            }
            mbedtls_sha256_update(&gSha, packet.bytes, packet.length);
            gWrittenOffset += packet.length;
            recordOtaActivity();
            publishStatus(String("ACK;offset=") + gWrittenOffset);
        }
    }

    if (!gCommitRequested) return;
    gCommitRequested = false;
    if (!gWriterActive || !gTargetConfirmed || gWrittenOffset != gDeclaredSize || gAcceptedOffset != gDeclaredSize || gPacketCount != 0) {
        publishStatus("ERROR;incomplete"); abortTransfer("incomplete"); return;
    }
    if (!gConnected || !TorqueDollyHandset::isMaintenanceActive()) {
        abortTransfer("disconnect-before-finalize"); return;
    }
    uint8_t digest[32];
    mbedtls_sha256_finish(&gSha, digest);
    char actual[65] = {};
    for (size_t i = 0; i < sizeof(digest); ++i) snprintf(&actual[i * 2], 3, "%02x", digest[i]);
    if (!String(actual).equalsIgnoreCase(gExpectedSha256)) {
        publishStatus("ERROR;sha256"); abortTransfer("sha256-mismatch"); return;
    }

    Preferences prefs;
    if (!prefs.begin(kPreferencesNamespace, false)) {
        publishStatus("ERROR;identity-storage"); abortTransfer("identity-storage"); return;
    }
    const bool metadataSaved = prefs.putUInt(kImageSizeKey, gDeclaredSize) == sizeof(uint32_t) &&
        prefs.putUInt(kImageIdentityKey, kImageIdentityTag) == sizeof(uint32_t) &&
        prefs.putString(kImageSha256Key, actual) == kHashHexLength;
    prefs.end();
    if (!metadataSaved) {
        restoreIdentityMetadata(); publishStatus("ERROR;identity-storage"); abortTransfer("identity-storage"); return;
    }

    // This critical section is the single COMMIT/cancellation decision point.
    // Cancellation that won the lock first prevents Update.end(false); after
    // gFinalizing is set, disconnect and ABORT cannot revoke this COMMIT.
    portENTER_CRITICAL(&gMux);
    const bool canceledBeforeFinalize = gCancelRequested || !gConnected || !gSessionActive;
    if (!canceledBeforeFinalize) {
        gFinalizing = true;
        gSessionActive = false;
    }
    portEXIT_CRITICAL(&gMux);
    if (canceledBeforeFinalize) {
        restoreIdentityMetadata();
        publishStatus("ERROR;finalize-canceled");
        abortTransfer("finalize-canceled");
        return;
    }
    if (!Update.end(false)) {
        restoreIdentityMetadata();
        publishStatus("ERROR;finalize");
        abortTransfer("finalize");
        return;
    }
    gUpdateFinalized = true;

    // COMMIT was accepted at the synchronized point above. Do not let later
    // disconnect/ABORT events roll back a successfully selected boot slot.
    TorqueDollyHandset::endOtaTransfer();
    if (gShaActive) {
        mbedtls_sha256_free(&gSha);
        gShaActive = false;
    }
    setTextValue(gIdentity, actual, kSha256HexLength);
    publishStatus("VALIDATED;rebooting;maintenance=locked");
    delay(100);
    ESP.restart();
}

#endif
