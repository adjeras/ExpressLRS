#pragma once

#if defined(TORQUE_DOLLY_TIMING_DIAGNOSTICS)

#include <stdint.h>

void timingDiagnosticsInit();
void timingDiagnosticsSetExpectedInterval(uint32_t intervalUs);
void timingDiagnosticsRecordTimerCallback();
void timingDiagnosticsRecordJoystickHandler();
void timingDiagnosticsReport(uint32_t nowMs);

#else

inline void timingDiagnosticsInit() {}
inline void timingDiagnosticsSetExpectedInterval(uint32_t) {}
inline void timingDiagnosticsRecordTimerCallback() {}
inline void timingDiagnosticsRecordJoystickHandler() {}
inline void timingDiagnosticsReport(uint32_t) {}

#endif
