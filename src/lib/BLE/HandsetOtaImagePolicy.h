#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// The ExpressLRS app image embeds its compiled target identity as:
// 0xBEEFCAFE, TARGET_NAME bytes, NUL. Do not infer image ownership from the
// shared ESP32-S3 chip ID alone.
namespace HandsetOtaImagePolicy {
inline bool validEsp32S3Header(const uint8_t *image, size_t size) {
    if (image == nullptr || size < 24 || image[0] != 0xE9 || image[1] == 0 || image[1] > 16) return false;
    const uint16_t chipId = static_cast<uint16_t>(image[12]) |
        (static_cast<uint16_t>(image[13]) << 8);
    return chipId == 0x0009;
}

inline bool containsExactTargetMarker(const uint8_t *preflight, size_t preflightSize,
                                      const uint8_t *targetMarker, size_t markerSize) {
    if (preflight == nullptr || targetMarker == nullptr || markerSize < 6 ||
        preflightSize < markerSize ||
        targetMarker[0] != 0xBE || targetMarker[1] != 0xEF ||
        targetMarker[2] != 0xCA || targetMarker[3] != 0xFE ||
        targetMarker[markerSize - 1] != 0) return false;
    for (size_t offset = 0; offset <= preflightSize - markerSize; ++offset) {
        if (memcmp(preflight + offset, targetMarker, markerSize) == 0) return true;
    }
    return false;
}
}
