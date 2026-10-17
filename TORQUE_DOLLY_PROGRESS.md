# Torque Dolly Phase B Progress

## Hardware and build context

- Hardware: ESP32-S3 Super Mini with external SX1280 F27 transmitter.
- Environment: `Unified_ESP32S3_2400_TX_via_UART`
- Unified configuration: `diy.tx_2400.torque-dolly-s3-f27`
- Current feature flags:
  - `-D TORQUE_DOLLY_RF_TEST=1`
  - `-D TORQUE_DOLLY_HANDSET=1`
  - `-D TORQUE_DOLLY_TIMING_DIAGNOSTICS=1`
  - `-D TORQUE_DOLLY_BLE_TELEMETRY=1`

## Phase B1: fixed BLE identification

### Hardware result: passed

- `TD-TX-BLE-B1` was read successfully across three reconnect cycles.
- Motor control worked normally with BLE connected.
- A fresh BLE read succeeded after motor operation.
- Timing during the motor test was 3978–4021 us, with an average of approximately 4000 us.
- `over10=0` during the motor test.

## Phase B2: live read-only status and uptime

### Hardware result: passed for observed behavior

- Live status characteristic works:
  - UUID: `7f2a0003-6d4e-4c8a-9b11-2e4d7a8f3101`
- Repeated reads showed increasing uptime.
- `LOCKED -> READY -> LOCKED` was verified.
- Phone screenshots verified `READY -> CONTROLLED_STOP -> LOCKED`.
- `READY` was read while the motor was rotating forward.
- A subsequent `LOCKED` read succeeded.
- Idle timing was 3987–4013 us, with an average of approximately 4000 us.
- During the motor-operation test with the serial monitor open, forward and reverse control operated normally and repeated BLE status reads worked normally.
- Motor-operation timing: `samples=161219`, `min_us=3987`, `max_us=4014`, `avg_us=4000`, `over10=0`.
- Motor-operation core assignment: `timer_core=1`, `joystick_core=1`.
- Motor-operation heap: `heap=193448`, `heap_min=193140` bytes.

### Combined Android app + transmitter + two-drive test

- The user reported normal control and continuing transmitter uptime updates.
- Timing diagnostics:
  - `rate=6`
  - `interval_us=4000`
  - `samples=121219`
  - `min_us=3986`
  - `max_us=4015`
  - `avg_us=3999`
  - `over10=0`
  - `timer_core=1`
  - `joystick_core=1`
  - `heap=193448`
  - `heap_min=193140`
- These observations do not claim individual verification of every drive telemetry field or every activation/locking step.
- `CONTROLLED_STOP` can retain nonzero output while the handset decelerates; `LOCKED` is the neutral-output state.

### Transmitter power-loss and recovery test

#### Hardware result: passed for the observed power-loss path

- The Android app was connected and the system slider remained `SYSTEM READY` throughout.
- The transmitter was `READY`, the stick was fully forward, and both motors were running at full speed.
- USB power was removed from the transmitter.
- Both motors decelerated progressively to a stop.
- The transmitter card changed to `CONNECTING`, then `RETRY EXHAUSTED`.
- Both physical receivers showed blinking blue LEDs.
- After transmitter power was restored, the card returned to `CONNECTED / LOCKED` with advancing uptime.
- The motors remained stopped until the user deliberately switched the transmitter to `READY` and moved the stick.
- Motion resumed normally without cycling the app slider.
- No automatic system lock or mandatory app reactivation was demonstrated.

The observed stop is attributed to the custom receiver-side ELRS failsafe implementation, not the transmitter-local `CONTROLLED_STOP` state or the VESC bridge `SAFELOCK` path.

Receiver implementation reference:

- Workspace: `C:/Users/Super/Documents/Arduino/expresslrs-rx-work`
- Branch: `feature/torque-dolly-rx-failsafe`
- Implementation commit: `6b1ecf1dfcc0fc73f633a4ca4c371e04ff6ad55f`
- Source: `src/lib/ServoOutput/devServoOutput.cpp`
- The receiver RF-loss PWM ramp is separate from transmitter `CONTROLLED_STOP` and bridge `SAFELOCK`.
- The exact firmware commit installed on the physical receivers remains unconfirmed.

This successful test covered transmitter power removal. RF loss while the transmitter remains powered and BLE stays connected was deferred and remains unverified.

### Remaining B2 test gaps

- RF range remains unqualified.
- Packet-delivery quality remains unqualified.
- RF loss while the transmitter remains powered and BLE stays connected remains unverified.
- Failsafe behavior during BLE activity remains unqualified.

## Next test

The following remain unverified:

- RF range.
- Packet-delivery quality.
- RF loss while the transmitter remains powered and BLE stays connected.
- Failsafe behavior during BLE activity.

This document records hardware observations only; it does not establish RF range, packet-delivery, or failsafe qualification.
