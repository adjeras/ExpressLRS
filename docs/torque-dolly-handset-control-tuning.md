# Torque Dolly Handset Control-Tuning Reference

Reference baseline: commit `9072939c`.

This document records the current Torque Dolly handset implementation and
current tuning rationale. It is a technical reference, not a production
certification or completed safety-validation report.

## Hardware and reference calibration

Configured ADC inputs and reference values are defined in
`src/lib/Handset/TorqueDollyControlParameters.h`:

| Quantity | X axis | Y axis |
|---|---:|---:|
| ADC pin | GPIO 1 | GPIO 2 |
| Reference minimum | 64 | 64 |
| Reference center | 1938 | 1925 |
| Reference maximum | 4095 | 4095 |
| Resolution | 12-bit | 12-bit |
| Attenuation | `ADC_11db` | `ADC_11db` |

The reference centers are initial expected values. At startup, the firmware
collects samples for 500 ms in ten 50 ms windows. It requires at least 100
valid samples and checks window population, finite statistics, center
proximity, standard deviation, span, drift, and adjacent-window changes.

When accepted, the runtime center is the median of the ten window means. The
filtered values are reset to the accepted centers. If acceptance fails, the
firmware retries after 500 ms.

The X axis is inverted once after calibration and normalization. Raw and
filtered ADC telemetry are not inverted.

## Filtering, normalization, and deadband

Each ADC axis uses an exponential moving average:

```text
filtered = 0.20 * raw + 0.80 * previous_filtered
```

Normalization is asymmetric around the runtime center:

```text
raw < center:
    -100 * (center - raw) / (center - minimum)

raw >= center:
    +100 * (raw - center) / (maximum - center)
```

The existing 2% deadband is then applied independently to each axis. Values
inside the deadband become exactly zero. Values outside it are rescaled so the
remaining range again reaches ±100%.

Safety, neutral, ready-state, inactivity, and related activity decisions use
the original normalized inputs. Later command shaping does not replace those
safety inputs.

## Response curves

The following table inputs are **post-deadband values**, before piecewise
linear response shaping. The outputs are signed symmetrically for the negative
half of each axis.

### Throttle response curve

| Post-deadband input | Output |
|---:|---:|
| 0 | 0 |
| 10 | 2 |
| 20 | 6 |
| 30 | 12 |
| 40 | 20 |
| 50 | 30 |
| 60 | 42 |
| 70 | 56 |
| 80 | 70 |
| 90 | 85 |
| 100 | 100 |

### Moving-steering response curve

| Post-deadband input | Output |
|---:|---:|
| 0 | 0 |
| 10 | 8 |
| 20 | 18 |
| 30 | 30 |
| 40 | 43 |
| 50 | 55 |
| 60 | 67 |
| 70 | 77 |
| 80 | 86 |
| 90 | 93 |
| 100 | 100 |

Both curves preserve sign, use piecewise linear interpolation, and saturate at
±100%.

## Command shaping and steering blend

After normalization, X inversion, and deadband:

1. Keep the deadbanded X value.
2. Apply rounded-gate compensation to the deadbanded Y value.
3. Apply the existing throttle response curve to Y, producing the shaped
   throttle command.
4. Apply the original response curve to deadbanded X.
5. Apply the moving-steering response curve to the same deadbanded X.
6. Blend the two steering commands.

The blend is:

```text
blend = constrain(abs(shaped_Y) / 40, 0, 1)
X_command = original_X + blend * (moving_X - original_X)
```

The blend ends at 40% shaped throttle command. It is based on the shaped
throttle **COMMAND**, not measured vehicle speed. `yDrive` is a slew-limited
throttle command used later by the control algorithm; it is not a measured
speed. The blend does not use measured vehicle speed.

The blended X command is not passed through the original response curve again.

When Ready gating or calibration gating is active, the shaped command values
are subsequently zeroed before slew and output processing. Consequently,
`xCommand` and `yCommand` telemetry are not always the ungated shaped joystick
values.

## Rounded-gate throttle compensation

The current compensation is bounded, approximate, throttle-only compensation
for a rounded joystick gate. It is not an exact inverse mapping and is not a
fitted gate calibration.

Using deadbanded X and Y:

```text
steeringFraction = constrain(abs(X) / 100, 0, 1)

gain = min(
    1.25,
    1 / sqrt(1 - 0.5 * steeringFraction²)
)

Y_compensated = constrain(Y * gain, -100, 100)
```

The compensation is symmetric for forward and reverse. X is unchanged. Since
deadband is applied first, zero throttle remains exactly zero and pure steering
does not create throttle.

## Slew limiting and reversal behavior

Throttle acceleration uses 24%/s. Throttle deceleration is blended between
30%/s and 70%/s according to `releaseFraction`:

```text
releaseFraction =
    (abs(yDrive) - abs(yTarget)) / abs(yDrive)

yDecelerationRate =
    30 + (70 - 30) * releaseFraction
```

The value is constrained to the valid range. A larger reduction in the target
therefore uses a faster deceleration. A throttle direction reversal uses the
70%/s stop rate.

Steering uses speed-blended rates, where `speedMagnitude = abs(yDrive)` is a
slew-limited throttle command, not measured vehicle speed:

| Condition | Acceleration | Deceleration |
|---|---:|---:|
| Standstill | 60%/s | 120%/s |
| Moving endpoint | 90%/s | 120%/s |

The steering rates blend continuously as `abs(yDrive)` rises from zero to
`PIVOT_BLEND_END`, which is 40%.

During a direction reversal, the slew helper first reduces the existing command
to zero without crossing it. The new direction then accelerates from zero.
There is no intentional output step or minimum-output boost.

## Steering authority

Current parameters are:

```text
FULL_STEERING_AUTHORITY = 70%
PIVOT_TURN_MAX           = 40%
PIVOT_BLEND_END          = 40%
MIN_STEERING_AUTHORITY   = 30%
```

The low-speed limit is:

```text
lowSpeedSteeringLimit =
    abs(yDrive) >= PIVOT_BLEND_END
        ? FULL_STEERING_AUTHORITY
        : PIVOT_TURN_MAX +
          (FULL_STEERING_AUTHORITY - PIVOT_TURN_MAX) *
          abs(yDrive) / PIVOT_BLEND_END
```

The high-speed limit is:

```text
highSpeedSteeringAuthority =
    FULL_STEERING_AUTHORITY -
    (FULL_STEERING_AUTHORITY - MIN_STEERING_AUTHORITY) *
    abs(yDrive) / DRIVE_PERCENT_RANGE
```

The applied authority is the minimum of both limits:

```text
steeringAuthority = min(
    lowSpeedSteeringLimit,
    highSpeedSteeringAuthority
)
```

Effective steering is:

```text
effectiveX = xDrive * steeringAuthority / 100
```

At standstill, authority is limited by the 40% pivot limit. At full
`abs(yDrive)`, the high-speed limit reaches 30%. The moving response curve
changes the requested steering command; it does not increase these authority
limits.

## Motor mixing and saturation

After slew limiting and authority calculation:

```text
leftDrive  = yDrive + effectiveX
rightDrive = yDrive - effectiveX
```

If either output exceeds ±100%, both outputs are scaled by the same factor:

```text
scale = 100 / max(abs(leftDrive), abs(rightDrive))
```

This preserves the relative mix but can reduce the available forward or reverse
output on one side during steering.

The resulting values are mapped independently to CRSF channel values. All
channels are first set to CRSF midpoint, then:

```text
ChannelData[0] = leftChannel
ChannelData[1] = rightChannel
```

Command percentages in diagnostics and telemetry are control-command values.
They are not measurements of motor torque, motor current, wheel speed, or
vehicle speed.

## Ready/Locked, inactivity, and power-button behavior

The handset starts Locked. The ready switch is debounced for 30 ms. A switch
transition away from the current Ready electrical state while Ready enters
ControlledStop. After the slew-limited outputs reach zero, the state becomes
Locked.

A deliberate transition is required to enter Ready. The joystick must be
neutral, with both original normalized axes inside the 2% deadband. This also
applies after inactivity locking and when startup finds the switch already in
its Ready electrical state.

The inactivity lock occurs after two minutes of neutral joystick time while
Ready. A non-neutral joystick excursion must qualify for 100 ms before it
resets the inactivity epoch. Shorter, unqualified excursions do not
necessarily reset the existing epoch.

The power button is active-low, debounced for 30 ms, and uses a three-second
hold followed by release for shutdown. Startup-held input must first be
released through debounce before a later hold can request shutdown. A
shutdown-pending state blinks the power-button feedback LED. Release after
shutdown becomes pending stops the transmit timer, enters the disconnected
state, and releases the power-hold output.

The automatic power-off firmware default is three minutes. The effective value
is loaded from persistent storage when available and can be adjusted from one
to sixty minutes. The saved value can therefore differ from the default.

The timer qualifies activity using a non-neutral joystick period of 100 ms. It
is suspended by power-button/shutdown activity, firmware update activity, and
excluded connection states. It also does not run as a normal idle timer until
calibration has succeeded. When the effective duration elapses during an
eligible neutral period, the handset publishes and consumes a shutdown request
and performs the same shutdown sequence as the power button.

## Telemetry fields

When BLE telemetry is enabled, the handset publishes:

| Field | Meaning |
|---|---|
| `xRaw`, `yRaw` | Most recent raw ADC readings |
| `xFiltered`, `yFiltered` | EMA-filtered ADC readings |
| `xCenter`, `yCenter` | Current runtime calibration centers |
| `xCommand`, `yCommand` | Command values after shaping, subject to later Ready/calibration gating |

The X command includes X-axis inversion. The Y command includes throttle gate
compensation and the throttle response curve. These values do not include slew
state, steering authority, motor mixing, output desaturation, or CRSF channel
mapping. When the handset is Locked or calibration has not succeeded, the
command values are zeroed by gating.

The serial diagnostic line additionally reports `X_DRIVE`, `Y_DRIVE`, steering
authority, effective X, left/right mixed outputs, and CRSF channels. `Y_DRIVE`
is a slew-limited command, not a measured vehicle speed.

BLE is telemetry/app observation. ELRS/CRSF is the vehicle-control path. BLE
connection state does not by itself establish the state of ELRS vehicle
control.

## Rationale for the current tuning

The current configuration reflects incremental tuning:

- X inversion corrected the physically reversed steering direction.
- Standstill steering acceleration was reduced from 200%/s to 60%/s for a
gentler pivot buildup.
- Standstill steering deceleration was increased from 50%/s to 120%/s so
steering is removed more promptly after release.
- Bounded throttle-only compensation was added to reduce the forward-command
loss observed when the rounded joystick gate is held at a steering angle.
- A separate moving-steering curve was added to make steering more responsive
near center while travelling, while preserving the original standstill feel.
- Authority limits, throttle shaping, slew behavior, safety checks, mixing,
and channel mapping were deliberately retained.

Amir reports excellent driving feel with the latest configuration.

## Validation status and open issues

The current status is:

- Amir's latest driving feedback: excellent feel.
- Beach and sand testing: pending.
- Feedback from previous wheelchair testers: pending.
- BLE handset connection intermittently reports `CONNECTING`, `RETRYING`,
  connection timeout, or `UNKNOWN`; root cause is unresolved.
- A previously reported brief unintended turn has not been diagnosed.
- No production certification is claimed.
- Complete safety validation is not claimed.

The baseline for this reference is commit `9072939c`.
