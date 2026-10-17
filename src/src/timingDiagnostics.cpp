#include "targets.h"

#if defined(TORQUE_DOLLY_TIMING_DIAGNOSTICS) && defined(TARGET_TX) && defined(PLATFORM_ESP32)

#include "timingDiagnostics.h"
#include "common.h"
#include "config.h"
#include "logging.h"

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

namespace {

struct TimingSnapshot {
    uint32_t expectedIntervalUs;
    uint32_t minIntervalUs;
    uint32_t maxIntervalUs;
    uint32_t lastTimerUs;
    uint64_t intervalSumUs;
    uint32_t over10Percent;
    uint32_t samples;
    int8_t timerCore;
    int8_t joystickCore;
    uint32_t heapFree;
    uint32_t heapMinimum;
};

portMUX_TYPE diagnosticsMux = portMUX_INITIALIZER_UNLOCKED;
TimingSnapshot diagnostics = {};
uint32_t lastReportMs;

} // namespace

void timingDiagnosticsInit()
{
    portENTER_CRITICAL(&diagnosticsMux);
    diagnostics = {};
    diagnostics.minIntervalUs = UINT32_MAX;
    diagnostics.timerCore = -1;
    diagnostics.joystickCore = -1;
    diagnostics.heapMinimum = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);
    portEXIT_CRITICAL(&diagnosticsMux);
    lastReportMs = millis();
}

void timingDiagnosticsSetExpectedInterval(uint32_t intervalUs)
{
    portENTER_CRITICAL(&diagnosticsMux);
    diagnostics.expectedIntervalUs = intervalUs;
    portEXIT_CRITICAL(&diagnosticsMux);
}

void IRAM_ATTR timingDiagnosticsRecordTimerCallback()
{
    const uint32_t nowUs = micros();
    portENTER_CRITICAL_ISR(&diagnosticsMux);

    if (diagnostics.lastTimerUs != 0) {
        const uint32_t intervalUs = nowUs - diagnostics.lastTimerUs;
        if (intervalUs < diagnostics.minIntervalUs) diagnostics.minIntervalUs = intervalUs;
        if (intervalUs > diagnostics.maxIntervalUs) diagnostics.maxIntervalUs = intervalUs;
        diagnostics.intervalSumUs += intervalUs;
        diagnostics.samples++;
        if (diagnostics.expectedIntervalUs != 0 &&
            intervalUs > diagnostics.expectedIntervalUs + diagnostics.expectedIntervalUs / 10) {
            diagnostics.over10Percent++;
        }
    }

    diagnostics.lastTimerUs = nowUs;
    diagnostics.timerCore = (int8_t)xPortGetCoreID();
    portEXIT_CRITICAL_ISR(&diagnosticsMux);
}

void timingDiagnosticsRecordJoystickHandler()
{
    portENTER_CRITICAL(&diagnosticsMux);
    diagnostics.joystickCore = (int8_t)xPortGetCoreID();
    portEXIT_CRITICAL(&diagnosticsMux);
}

void timingDiagnosticsReport(uint32_t nowMs)
{
    if ((uint32_t)(nowMs - lastReportMs) < 5000) return;
    lastReportMs = nowMs;

    TimingSnapshot snapshot;
    portENTER_CRITICAL(&diagnosticsMux);
    snapshot = diagnostics;
    portEXIT_CRITICAL(&diagnosticsMux);
    snapshot.heapFree = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    snapshot.heapMinimum = heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT);

    const uint32_t averageUs = snapshot.samples == 0
        ? 0
        : (uint32_t)(snapshot.intervalSumUs / snapshot.samples);

    Serial.printf("TIMING rate=%u interval_us=%u samples=%u min_us=%u max_us=%u avg_us=%u over10=%u timer_core=%d joystick_core=%d heap=%u heap_min=%u\n",
                  config.GetRate(), snapshot.expectedIntervalUs, snapshot.samples,
                  snapshot.samples == 0 ? 0 : snapshot.minIntervalUs,
                  snapshot.maxIntervalUs, averageUs, snapshot.over10Percent,
                  snapshot.timerCore, snapshot.joystickCore,
                  snapshot.heapFree, snapshot.heapMinimum);
}

#endif
