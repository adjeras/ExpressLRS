#pragma once

#include <Arduino.h>

namespace TorqueDollyControlParameters
{
// -----------------------------------------------------------------------------
// Hardware and Hall joystick calibration
// -----------------------------------------------------------------------------
constexpr int PIN_X = 1;
constexpr int PIN_Y = 2;
constexpr int PIN_STATUS_LED = 43;

// Initial direct-3.3V joystick references from observed ADC endpoints.
// The low endpoint margin is conservative below the measured minimum; the
// high endpoint reaches the 12-bit ADC ceiling at the mechanical endpoint.
constexpr int X_MIN = 64;
constexpr int X_CENTER = 1938;
constexpr int X_MAX = 4095;

constexpr int Y_MIN = 64;
constexpr int Y_CENTER = 1925;
constexpr int Y_MAX = 4095;

constexpr int ADC_RESOLUTION_BITS = 12;
constexpr adc_attenuation_t ADC_ATTENUATION = ADC_11db;

constexpr unsigned long STARTUP_CALIBRATION_DURATION_MS = 500;
constexpr unsigned long STARTUP_CALIBRATION_RETRY_DELAY_MS = 500;
constexpr int STARTUP_CALIBRATION_CENTER_WINDOW_RAW = 60;
constexpr int STARTUP_CALIBRATION_MAX_DEVIATION_RAW = 20;
constexpr float STARTUP_CALIBRATION_MAX_STDDEV_RAW = 12.0f;
// Temporary calibration experiment: use 36 instead of the previous 24 threshold.
constexpr float STARTUP_CALIBRATION_WINDOW_MAX_STDDEV_RAW = 36.0f;
constexpr unsigned long STARTUP_CALIBRATION_MIN_SAMPLES = 100;
constexpr uint8_t STATUS_LED_CALIBRATION_BLUE = 64;
constexpr unsigned long CALIBRATION_FAILURE_BLINK_PERIOD_MS = 500;
constexpr unsigned long CALIBRATION_FAILURE_RED_MS = 250;

// -----------------------------------------------------------------------------
// Joystick filtering and response
// -----------------------------------------------------------------------------
constexpr float EMA_ALPHA = 0.20f;
constexpr float DEADBAND = 2.0f;

constexpr float RESPONSE_CURVE_INPUT[] = {
    0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100
};

constexpr float RESPONSE_CURVE_OUTPUT[] = {
    0, 2, 6, 12, 20, 30, 42, 56, 70, 85, 100
};

constexpr float STEERING_RESPONSE_CURVE_OUTPUT[] = {
    0, 8, 18, 30, 43, 55, 67, 77, 86, 93, 100
};

constexpr float STEERING_RESPONSE_BLEND_END = 40.0f;
constexpr int RESPONSE_CURVE_SEGMENT_COUNT = 10;
constexpr float DRIVE_PERCENT_MIN = -100.0f;
constexpr float DRIVE_PERCENT_MAX = 100.0f;
constexpr float DRIVE_PERCENT_RANGE = 100.0f;

// Bounded, approximate throttle-only compensation for a rounded joystick
// gate. This is not an exact inverse mapping or a fitted calibration.
constexpr float THROTTLE_GATE_COMPENSATION_FACTOR = 0.5f;
constexpr float THROTTLE_GATE_MAX_GAIN = 1.25f;

// -----------------------------------------------------------------------------
// Acceleration and deceleration
// -----------------------------------------------------------------------------
constexpr float ACCEL_RATE = 24.0f;
constexpr float DECEL_RATE_GENTLE = 30.0f;
constexpr float DECEL_RATE_STOP = 70.0f;

// -----------------------------------------------------------------------------
// Steering and tank mixing
// -----------------------------------------------------------------------------
constexpr float FULL_STEERING_AUTHORITY = 70.0f;
constexpr float PIVOT_TURN_MAX = 40.0f;
constexpr float PIVOT_BLEND_END = 40.0f;
constexpr float STEERING_ACCEL_RATE_STANDSTILL = 60.0f;
constexpr float STEERING_ACCEL_RATE_MOVING = 90.0f;
constexpr float STEERING_DECEL_RATE_STANDSTILL = 120.0f;
constexpr float STEERING_DECEL_RATE_MOVING = 120.0f;
constexpr float MIN_STEERING_AUTHORITY = 30.0f;

// -----------------------------------------------------------------------------
// Ready/Locked state machine and status LED
// -----------------------------------------------------------------------------
constexpr unsigned long BUTTON_DEBOUNCE_MS = 30;
constexpr unsigned long BUTTON_UNLOCK_MIN_PRESS_MS = 700;
constexpr unsigned long BUTTON_SHUTDOWN_PRESS_MS = 3000;
constexpr unsigned long INACTIVITY_LOCK_MS = 120000;
constexpr uint8_t DEFAULT_AUTO_POWER_OFF_MINUTES = 3;
constexpr uint8_t AUTO_POWER_OFF_MINUTES_MIN = 1;
constexpr uint8_t AUTO_POWER_OFF_MINUTES_MAX = 60;
constexpr unsigned long ACTIVITY_QUALIFICATION_MS = 100;
constexpr unsigned long DIAGNOSTIC_INTERVAL_MS = 100;
constexpr unsigned long INACTIVITY_BLINK_PERIOD_MS = 1000;
constexpr unsigned long INACTIVITY_BLINK_RED_MS = 100;
constexpr unsigned long BUTTON_FEEDBACK_BLINK_PERIOD_MS = 200;
constexpr unsigned long BUTTON_FEEDBACK_BLINK_ON_MS = 100;
constexpr unsigned long AUTO_POWER_OFF_DIAGNOSTIC_INTERVAL_MS = 5000;
constexpr uint8_t STATUS_LED_BRIGHTNESS_PERCENT = 25;

constexpr uint8_t STATUS_LED_RED = 64;
constexpr uint8_t STATUS_LED_PINK_RED = 64;
constexpr uint8_t STATUS_LED_PINK_BLUE = 64;
constexpr uint8_t STATUS_LED_GREEN = 64;
constexpr uint8_t STATUS_LED_AMBER_RED = 64;
constexpr uint8_t STATUS_LED_AMBER_GREEN = 32;

// -----------------------------------------------------------------------------
// Control timing
// -----------------------------------------------------------------------------
constexpr float MAX_CONTROL_DT_SECONDS = 0.1f;

} // namespace TorqueDollyControlParameters
