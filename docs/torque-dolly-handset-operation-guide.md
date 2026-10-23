# Torque Dolly Handset Operation Guide

Reference baseline: commit `9072939c`.

This guide describes behavior implemented by the current Torque Dolly handset
firmware. It does not claim production certification or completed safety
validation.

## Starting the handset

At startup, keep the joystick at its center position. The handset samples the
joystick for a short calibration period and checks that the readings are stable
and close to the expected center. During calibration, the status LED shows blue.

If calibration is accepted, the handset uses the measured center for control.
If it is not accepted, it retries after a delay. Keep the joystick still and
centered until calibration succeeds.

The power button is sampled as an active-low input. The handset's power-hold
output is asserted during startup so the unit remains powered. A button that is
already held while the handset starts is treated as a startup-held button; it
cannot immediately request shutdown until a debounced release has occurred.

## Ready/Locked operation

The handset starts in Locked and vehicle commands are neutral while Locked.
The ready switch is debounced for 30 ms.

### Startup with the switch already in Ready

If the switch is already in its Ready electrical state at startup, the handset
does not immediately enter Ready. It starts Locked and requires a deliberate
switch transition before it can become Ready.

Use this sequence:

1. Keep the joystick centered during startup calibration.
2. Move the ready switch away from its startup Ready state and allow the
   debounced transition to be recognized.
3. Return the switch to its startup Ready state while the joystick remains
   neutral.
4. After the return transition is debounced and the startup-neutral condition
   has cleared, the handset enters Ready.

This guide intentionally describes the sequence without assigning physical
labels such as up/down, because the firmware verifies electrical states rather
than a physical switch orientation.

### Normal Locked-to-Ready sequence

When Locked with the switch in its non-Ready electrical state, move the switch
to the Ready electrical state and keep the joystick neutral. The firmware
requires the transition to be debounced and the normalized joystick inputs to
be within the 2% deadband before entering Ready.

Moving the switch out of the Ready electrical state while Ready enters
ControlledStop. The current drive commands are brought to zero through the
existing slew behavior; the handset then returns to Locked.

### Recovery after inactivity locking

After inactivity locking, the handset is Locked and the inactivity indicator is
active. To recover:

1. Keep the joystick neutral.
2. Move the ready switch away from its Ready electrical state and wait for the
   debounced transition.
3. Return the switch to the Ready electrical state while keeping the joystick
   neutral.
4. The handset can then enter Ready after the deliberate transition is
   recognized.

Do not move the joystick during this sequence. The same deliberate transition
requirement prevents an already-held switch position from silently re-enabling
vehicle commands.

When Ready, joystick movement can produce vehicle commands. When Locked,
vehicle commands are neutralized. The status LED is green in Ready and red in
normal Locked operation. During ControlledStop it is amber. After inactivity
locking, the Locked indication alternates green and red according to the
inactivity blink timing.

## Driving forward and reverse

Move the Y axis away from center to request forward or reverse travel. The
handset applies a gradual acceleration rather than an immediate full command.

Returning the Y axis toward center reduces the command progressively. A
forward-to-reverse change first decelerates the current drive command through
zero, then builds in the new direction.

The displayed command percentages are control-command values. They are not
measurements of motor torque, vehicle speed, or wheel speed.

## Steering while travelling

Move the X axis to steer while the vehicle is travelling. The physical X axis
is corrected in firmware so the resulting steering direction matches the
vehicle direction.

Steering response becomes more direct as the shaped throttle command rises.
This uses a shaped throttle command, not measured vehicle speed and not a
measurement of `yDrive` as physical speed. Steering authority is also limited
by the normal drive-mixing behavior.

Forward/reverse travel and steering are combined into separate left and right
vehicle commands. When a combined command reaches the available output range,
the outputs are proportionally reduced to remain bounded.

## Pivot turns at standstill

With the vehicle stopped, steering can request a pivot turn: one side is
commanded in one direction and the other side in the opposite direction.

Pivot authority is limited to the configured standstill pivot limit. The
steering command builds gradually and is removed more quickly when the stick
is released.

The handset does not add a minimum-output jump. If the motor or vehicle needs
more torque to overcome static friction, physical movement may begin only
above a visible command threshold.

## Stopping

Release the joystick gradually toward neutral for a controlled reduction in
vehicle command. Direction changes are also handled through zero rather than
instantaneously reversing the drive output.

Moving the ready switch out of the Ready electrical state initiates the
controlled-stop behavior when applicable. The vehicle commands are then
returned to neutral and the handset becomes Locked.

## Power button operation and LED feedback

The power button is active-low and debounced for 30 ms.

- A button press is recorded after debounce.
- Holding the button for three seconds creates a shutdown-pending state.
- While shutdown is pending, the status LED provides a blinking pink/red-blue
  feedback indication.
- Releasing the button after shutdown becomes pending powers the handset off
  by stopping the transmit timer, entering the disconnected state, and
  releasing the power-hold output.

A short press/release does not request shutdown. A button held during startup
must first be released through debounce before a later hold can request
shutdown.

LED meanings used by the handset are:

| LED indication | Meaning |
|---|---|
| Blue | Startup calibration in progress |
| Red | Locked |
| Green | Ready |
| Amber | ControlledStop |
| Alternating green/red | Locked by inactivity |
| Blinking pink/red-blue | Power-button shutdown pending |

Calibration failure uses a red blink pattern. The power-button feedback takes
priority while it is active.

## Inactivity locking

The inactivity lock occurs after two minutes of neutral joystick time while the
handset is Ready. A non-neutral joystick excursion must qualify for 100 ms
before it resets the inactivity epoch. Shorter, unqualified excursions do not
necessarily reset the existing epoch.

The handset also requires neutral input for its inactivity lock condition. When
locked by inactivity, the status indication identifies that condition and the
Ready/Locked recovery sequence above is required.

## Automatic power-off

The firmware default automatic power-off duration is three minutes. The actual
value is loaded from persistent storage when available and can be adjusted
through the supported BLE setting from one to sixty minutes. Therefore the
effective duration may differ from the three-minute default.

The auto-power-off timer is reset or suspended by current firmware conditions,
including:

- Joystick activity that remains non-neutral for the 100 ms activity
  qualification period.
- The power button being held or shutdown already being active.
- Firmware update activity.
- Connection states excluded by the handset's connection-state checks.
- Calibration not yet being complete.

When the timer reaches the effective configured duration during an eligible
neutral period, the handset publishes and consumes an automatic shutdown
request and performs the same power-off sequence as a button-triggered
shutdown.

Automatic power-off is separate from the vehicle's ordinary neutral command.
Do not rely on it as a substitute for putting the vehicle in a safe physical
state.

## ELRS vehicle control and BLE telemetry

ELRS is the vehicle-control path. The handset writes the left and right drive
commands to the configured CRSF channels for transmission to the vehicle.

BLE is a telemetry and app-observation path. It exposes joystick and command
information to the Android app, including raw and filtered readings, centers,
and command values. Those command values are safety-gated and can be zeroed
when the handset is Locked or calibration has not succeeded; they are not
always ungated shaped joystick values.

BLE telemetry is not the vehicle-control transport. A BLE app disconnect or
reconnect state must not be interpreted as proof that ELRS vehicle control is
available or unavailable.

Current reports include intermittent BLE states such as `CONNECTING`,
`RETRYING`, connection timeout, and `UNKNOWN`. The cause is unresolved.

## Current validation status

Amir reports excellent driving feel with the latest configuration. Beach and
sand testing, along with feedback from previous wheelchair testers, are
pending.

A previously reported brief unintended turn has not been diagnosed. No
production certification or complete safety validation is claimed by this
document.
