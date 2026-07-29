# Drive-Wheel Encoder Acquisition and Arduino Implementation

## Purpose

This document defines how the Arduino Mega 2560 must acquire and report the two drive-wheel encoders so that the Raspberry Pi 4B can perform drive-wheel odometry.

It is intended to provide enough detail for another developer to complete the encoder implementation in the Mega `.ino` firmware without having to repeat the architectural decisions.

Where several technically valid approaches exist, the alternatives are retained for context. The implementation required for this robot is explicitly identified as **Selected path for this robot**.

This document covers:

- the Pi-facing encoder IO contract;
- Arduino and Pi responsibilities;
- quadrature decoding;
- Mega pin allocation;
- timer-driven encoder sampling;
- encoder state and atomic snapshots;
- serial request and response handling;
- initial validity behaviour;
- encoder resolution and configuration;
- sign conventions;
- implementation and verification steps.

It does not implement the final Pi-side localisation or sensor-fusion logic.

---

## 1. Pi-Facing Encoder IO Model

Each physical encoder exposes four Pi IO properties:

| Property | Meaning |
|---|---|
| `count` | Signed cumulative quadrature count |
| `timestamp_ms` | Arduino time at which the returned encoder snapshot was taken |
| `valid` | Whether the returned encoder snapshot is usable |
| `valid_flags` | Bit field describing a current encoder validity condition |

The Pi IO points are:

```python
io.encoder["drive_front_left"].count
io.encoder["drive_front_left"].timestamp_ms
io.encoder["drive_front_left"].valid
io.encoder["drive_front_left"].valid_flags

io.encoder["drive_front_right"].count
io.encoder["drive_front_right"].timestamp_ms
io.encoder["drive_front_right"].valid
io.encoder["drive_front_right"].valid_flags
```

### Available approaches

1. Use a separate serial command for every Pi IO property.
2. Use one command per physical encoder and return all properties together.
3. Continuously stream encoder data without a request.

### Selected path for this robot

**Use one requested serial transaction per physical encoder. The response contains all four Pi IO properties.**

Request:

```text
ENCODER drive_front_left READ
```

Response:

```text
OK ENCODER drive_front_left count=12345 timestamp_ms=482761 valid=1 valid_flags=0
```

The Pi backend parses the named fields and updates the four corresponding IO properties.

The four spreadsheet rows for each encoder therefore represent four Pi properties derived from one serial response. They are not four separate Arduino measurements or four separate serial commands.

The same applies to the right encoder:

```text
ENCODER drive_front_right READ
```

---

## 2. Current Firmware Starting Point

The current Mega firmware already defines the intended pins:

```cpp
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_A  = 22;
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_B  = 24;

static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_A = 26;
static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_B = 28;
```

They are configured as inputs:

```cpp
pinMode(PIN_ENC_DRIVE_FRONT_LEFT_A, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_LEFT_B, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_A, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_B, INPUT_PULLUP);
```

The current placeholder function reads only the immediate A/B pin state:

```cpp
long readQuadPair(uint8_t pinA, uint8_t pinB) {
  int a = digitalRead(pinA) ? 1 : 0;
  int b = digitalRead(pinB) ? 1 : 0;
  return (a << 1) | b;
}
```

Its possible results are:

| A | B | Returned value |
|---:|---:|---:|
| 0 | 0 | 0 |
| 0 | 1 | 1 |
| 1 | 0 | 2 |
| 1 | 1 | 3 |

This function does not:

- accumulate movement;
- determine direction;
- timestamp a sample;
- detect invalid transitions;
- provide an odometry count.

### Selected path for this robot

**Retain `READ QUAD` only as an optional low-level wiring diagnostic. Do not use it as the backend for `io.encoder[...]`.**

The completed firmware must add cumulative quadrature decoding and implement:

```text
ENCODER <name> READ
```

---

## 3. Division of Responsibility

### Available approaches

1. Send raw A/B states to the Pi and perform all decoding there.
2. Decode and count on the Arduino; perform odometry on the Pi.
3. Perform encoder counting and complete robot pose calculation on the Arduino.

### Selected path for this robot

**The Arduino performs real-time quadrature acquisition and cumulative counting. The Pi converts count changes into wheel travel, robot motion and localisation.**

```text
Physical A/B signals
        ↓
Arduino Mega
- samples A and B
- decodes direction
- accumulates signed counts
- records diagnostics
- returns atomic snapshots
        ↓
Serial interface
        ↓
Raspberry Pi 4B
- calculates count deltas
- applies encoder polarity
- converts counts to wheel travel
- calculates translation and heading change
- fuses odometry with IMU, vision, deadwheels or OTOS
- maintains robot pose
```

### Arduino responsibilities

The Mega must:

- continuously sample both A and B channels;
- decode valid quadrature transitions;
- maintain one signed cumulative count per encoder;
- retain the previous A/B state;
- count invalid transitions diagnostically;
- return a coherent atomic encoder snapshot;
- expose the semantic `ENCODER <name> READ` interface.

### Pi responsibilities

The Pi must:

- calculate consecutive count differences;
- apply robot-specific encoder polarity;
- convert count deltas to wheel distance;
- calculate centre displacement and heading change;
- manage effective wheel diameter and track-width calibration;
- combine drive-wheel odometry with other localisation providers.

The Arduino must not calculate global `x`, `y` or heading.

---

## 4. Encoder State Structure

### Available approaches

1. Store each field in separate global variables.
2. Use Arduino `long` values without explicit sizes.
3. Use one explicitly typed structure for each encoder.

### Selected path for this robot

**Use one `EncoderState` structure per physical encoder. Use explicitly sized integer types.**

Recommended starting structure:

```cpp
struct EncoderState {
  volatile int32_t count;
  volatile uint32_t last_edge_us;
  volatile uint16_t invalid_transition_count;
  volatile uint8_t previous_ab;
  volatile bool initialized;
};
```

Instances:

```cpp
EncoderState encoderDriveFrontLeft;
EncoderState encoderDriveFrontRight;
```

Suggested initial values:

```cpp
EncoderState encoderDriveFrontLeft = {
  0,      // count
  0,      // last_edge_us
  0,      // invalid_transition_count
  0,      // previous_ab; replaced during setup
  false   // initialized
};

EncoderState encoderDriveFrontRight = {
  0,
  0,
  0,
  0,
  false
};
```

### Why `int32_t`

The Mega uses a 32-bit `long`, but `int32_t` makes the serial contract explicit and portable.

### Why fields are `volatile`

The timer interrupt updates these values while normal firmware code reads them. `volatile` prevents the compiler from assuming that they cannot change unexpectedly.

### Purpose of `last_edge_us`

`last_edge_us` is retained internally for future diagnostics, such as:

- confirming that an encoder is active;
- estimating speed;
- identifying implausibly fast transitions;
- investigating noisy signals.

It is not the current meaning of the public `timestamp_ms` property.

---

## 5. Meaning of `timestamp_ms`

### Available approaches

#### Option A: snapshot timestamp

```text
timestamp_ms = Arduino time when the returned count snapshot was taken
```

Advantages:

- represents the age of the complete returned sample;
- continues advancing while the wheel is stationary;
- suitable for checking serial-data freshness.

Disadvantage:

- does not directly identify the time of the last encoder edge.

#### Option B: last-edge timestamp

```text
timestamp_ms = Arduino time of the most recent valid encoder transition
```

Advantages:

- useful for activity, stall and speed diagnostics.

Disadvantages:

- remains old when a healthy wheel is stationary;
- may be incorrectly interpreted by the Pi as stale data.

#### Option C: Pi receive timestamp

The Pi timestamps the response when it arrives.

Disadvantage:

- describes serial receipt rather than when the Arduino captured the encoder state.

### Selected path for this robot

**`timestamp_ms` is the Arduino time when the returned encoder snapshot is taken.**

Conceptually:

```cpp
uint32_t timestampMs = millis();
```

A future property may separately expose:

```python
io.encoder["drive_front_left"].last_edge_timestamp_ms
```

That property is not required for the initial implementation.

---

## 6. Quadrature Decoding

A quadrature encoder produces two square-wave channels, A and B, offset in phase.

One direction normally follows:

```text
00 → 01 → 11 → 10 → 00
```

The opposite direction follows:

```text
00 → 10 → 11 → 01 → 00
```

### Available decoding resolutions

1. **x1** — count one selected edge of channel A.
2. **x2** — count both edges of channel A.
3. **x4** — count every valid A/B transition.

### Selected path for this robot

**Use x4 quadrature decoding.**

The firmware contract is:

```text
count = signed cumulative number of valid A/B state transitions
```

Each valid transition changes the count by `+1` or `-1`.

### Available decoding implementations

1. Nested conditionals examining A and B.
2. Interrupt only on A and inspect B for direction.
3. Previous/current-state transition lookup table.

### Selected path for this robot

**Use a 16-entry previous/current-state lookup table.**

Example:

```cpp
static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
```

Lookup:

```cpp
uint8_t transition = (state.previous_ab << 2) | current_ab;
int8_t delta = QUAD_TABLE[transition];
```

For a valid movement:

```cpp
state.count += delta;
state.last_edge_us = micros();
```

The previous state must always be updated after processing:

```cpp
state.previous_ab = current_ab;
```

### Invalid transitions

Transitions such as these change both bits between samples:

```text
00 → 11
01 → 10
11 → 00
10 → 01
```

They usually indicate that an intermediate transition was missed or that the signal was noisy.

For the initial implementation:

- do not add or subtract from `count`;
- increment `invalid_transition_count`;
- update `previous_ab` to the observed current state;
- do not make the encoder permanently invalid.

The transition table above returns zero for both an unchanged state and a two-bit jump. The implementation must therefore separately distinguish those cases if it is to increment `invalid_transition_count`.

A simple test is:

```cpp
uint8_t changed = state.previous_ab ^ current_ab;

bool bothBitsChanged = (changed == 0b11);
```

---

## 7. Mega Encoder Pins

The assigned pins are:

| Encoder | Channel | Mega pin | AVR port bit |
|---|---|---:|---|
| `drive_front_left` | A | 22 | `PA0` |
| `drive_front_left` | B | 24 | `PA2` |
| `drive_front_right` | A | 26 | `PA4` |
| `drive_front_right` | B | 28 | `PA6` |

### Available approaches

1. Move all encoder signals to external-interrupt pins.
2. Retain pins 22, 24, 26 and 28 and sample them using a timer.
3. Add an external quadrature-counter device.

The Mega's normal external interrupt pins are:

```text
2, 3, 18, 19, 20, 21
```

In the existing design:

- pins 18 and 19 are used by a RoboClaw serial connection;
- pins 20 and 21 are reserved for I²C;
- the wider pin allocation makes moving four encoder channels undesirable.

### Selected path for this robot

**Retain pins 22, 24, 26 and 28. Use timer-driven direct-port sampling.**

All four signals are on `PORTA`, so one register read captures them together:

```cpp
uint8_t portA = PINA;
```

This preserves the existing IO map and avoids four independent `digitalRead()` calls.

---

## 8. Encoder Sampling Method

### Available approaches

1. Poll in the ordinary Arduino `loop()`.
2. Use external interrupts.
3. Use a hardware timer interrupt.
4. Use an external quadrature counter.

### Why ordinary-loop polling is not suitable

The existing AUTO path includes a delay of approximately 20 ms:

```cpp
delay(20);
```

That limits the main loop to approximately 50 iterations per second before allowing for other work.

The loop also handles serial communication, motor commands, receiver input, servos and voltage measurement. Its timing is therefore neither fast enough nor deterministic enough for encoder edge capture.

### Selected path for this robot

**Use a hardware timer interrupt to sample the encoder pins at a fixed high rate.**

The timer interrupt must:

1. read `PINA` once;
2. extract left A and B;
3. extract right A and B;
4. form the current two-bit state for each encoder;
5. decode each transition;
6. update each encoder state;
7. return immediately.

No serial printing, floating-point arithmetic or odometry calculation may occur inside the timer interrupt.

---

## 9. Direct `PORTA` Reading

The relevant bits from `PINA` are:

```text
bit 0: PA0 / Mega pin 22 / left A
bit 2: PA2 / Mega pin 24 / left B
bit 4: PA4 / Mega pin 26 / right A
bit 6: PA6 / Mega pin 28 / right B
```

Example extraction:

```cpp
uint8_t portA = PINA;

uint8_t leftA  = (portA >> PA0) & 0x01;
uint8_t leftB  = (portA >> PA2) & 0x01;
uint8_t rightA = (portA >> PA4) & 0x01;
uint8_t rightB = (portA >> PA6) & 0x01;

uint8_t leftAB  = (leftA << 1) | leftB;
uint8_t rightAB = (rightA << 1) | rightB;
```

The exact A/B bit order may be reversed as long as it is used consistently. Reversing the order reverses the raw count direction, which is later handled by the Pi installation polarity.

### Selected path for this robot

**Read `PINA` once per timer interrupt and decode both encoders from that one captured port value.**

---

## 10. Timer Selection and Sampling Rate

The required sampling rate depends on the maximum encoder transition frequency.

For a published output-shaft count:

\[
f_{edge} = \frac{RPM_{output}}{60} \times counts_{output\ rev}
\]

For the DFRobot FIT0186 nominal values:

- output speed: approximately 251 rpm;
- nominal output count: approximately 700 counts/revolution;

the nominal maximum transition rate, if the 700 value matches the implemented x4 count convention, is approximately:

\[
\frac{251}{60} \times 700 \approx 2928\ transitions/second
\]

A timer sampler must run comfortably faster than the transition rate. Sampling merely at twice the edge rate is not sufficiently robust because edge timing is asynchronous to the timer.

### Available approaches

1. Select a sampling rate immediately from nominal manufacturer data.
2. First implement a conservative several-kilohertz rate and validate missed transitions.
3. Move to hardware interrupts or an external counter if timer polling proves insufficient.

### Selected path for this robot

**Use timer-driven polling and begin with a practical conservative rate selected after checking timer conflicts in the complete `.ino` file. Validate the rate experimentally before adding stricter fault handling.**

The exact hardware timer and compare value are not fixed in this document because they must be selected against the timers already used by:

- servo libraries;
- tone or audio functions;
- PWM outputs;
- other firmware timing requirements.

The developer completing the `.ino` file must:

1. inventory all timer use in the complete firmware;
2. select a free timer;
3. choose a several-kilohertz initial sample rate;
4. test one-revolution counts at slow and powered speeds;
5. increase the rate if counts decrease at higher speed;
6. keep the interrupt routine short.

The first objective is repeatable workable counts, not an elaborate real-time fault framework.

---

## 11. Encoder Initialisation

The initial A/B state must be captured before timer decoding begins. Otherwise the first sample may be interpreted as movement.

Recommended setup sequence:

```cpp
pinMode(PIN_ENC_DRIVE_FRONT_LEFT_A, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_LEFT_B, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_A, INPUT_PULLUP);
pinMode(PIN_ENC_DRIVE_FRONT_RIGHT_B, INPUT_PULLUP);

uint8_t portA = PINA;

encoderDriveFrontLeft.previous_ab = readLeftABFromPortA(portA);
encoderDriveFrontRight.previous_ab = readRightABFromPortA(portA);

encoderDriveFrontLeft.count = 0;
encoderDriveFrontRight.count = 0;

encoderDriveFrontLeft.invalid_transition_count = 0;
encoderDriveFrontRight.invalid_transition_count = 0;

encoderDriveFrontLeft.last_edge_us = micros();
encoderDriveFrontRight.last_edge_us = micros();

encoderDriveFrontLeft.initialized = true;
encoderDriveFrontRight.initialized = true;

setupEncoderSamplingTimer();
```

### Selected path for this robot

**Configure the pins, capture both initial A/B states, initialize both state structures and only then enable the sampling timer.**

---

## 12. Timer Interrupt Logic

Conceptual structure:

```cpp
ISR(TIMERn_COMPA_vect) {
  uint8_t portA = PINA;

  uint8_t leftAB = readLeftABFromPortA(portA);
  uint8_t rightAB = readRightABFromPortA(portA);

  updateEncoderFromAB(encoderDriveFrontLeft, leftAB);
  updateEncoderFromAB(encoderDriveFrontRight, rightAB);
}
```

Conceptual update function:

```cpp
inline void updateEncoderFromAB(
    volatile EncoderState &state,
    uint8_t currentAB) {

  uint8_t previousAB = state.previous_ab;

  if (currentAB == previousAB) {
    return;
  }

  uint8_t changed = previousAB ^ currentAB;

  if (changed == 0b11) {
    state.invalid_transition_count++;
    state.previous_ab = currentAB;
    return;
  }

  uint8_t transition = (previousAB << 2) | currentAB;
  int8_t delta = QUAD_TABLE[transition];

  state.count += delta;
  state.last_edge_us = micros();
  state.previous_ab = currentAB;
}
```

The final implementation may optimise this further once it works correctly.

### Initial design priority

**Prefer simple, testable and repeatable counting over premature optimisation or rigorous control logic.**

---

## 13. Atomic Encoder Snapshots

The Mega is an 8-bit microcontroller. Reading a 32-bit count can require several machine operations. The timer interrupt could update the value partway through a normal read.

### Available approaches

1. Read volatile values directly while printing.
2. Disable interrupts during the complete serial response.
3. Briefly disable interrupts, copy the state, restore interrupts and then print.

### Selected path for this robot

**Take a short atomic copy of the state and restore interrupts before serial printing.**

Example snapshot structure:

```cpp
struct EncoderSnapshot {
  int32_t count;
  uint16_t invalid_transition_count;
  bool initialized;
  uint32_t timestamp_ms;
};
```

Example function:

```cpp
EncoderSnapshot snapshotEncoder(const EncoderState &state) {
  EncoderSnapshot snapshot;

  noInterrupts();
  snapshot.count = state.count;
  snapshot.invalid_transition_count = state.invalid_transition_count;
  snapshot.initialized = state.initialized;
  snapshot.timestamp_ms = millis();
  interrupts();

  return snapshot;
}
```

`millis()` itself depends on interrupts on the AVR. A safer implementation is either:

1. copy `millis()` immediately before disabling interrupts; or
2. copy the encoder fields with `ATOMIC_BLOCK` and obtain `millis()` outside that block.

Recommended form:

```cpp
EncoderSnapshot snapshotEncoder(const EncoderState &state) {
  EncoderSnapshot snapshot;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    snapshot.count = state.count;
    snapshot.invalid_transition_count = state.invalid_transition_count;
    snapshot.initialized = state.initialized;
  }

  snapshot.timestamp_ms = millis();
  return snapshot;
}
```

This requires:

```cpp
#include <util/atomic.h>
```

The small difference between copying the count and calling `millis()` is acceptable for this initial encoder IO contract.

---

## 14. Serial Command Handling

The Pi must address encoders by semantic name, not physical pin number.

Supported requests:

```text
ENCODER drive_front_left READ
ENCODER drive_front_right READ
```

### Selected path for this robot

**Resolve the encoder name to its `EncoderState`, take one snapshot and return all public properties in one response.**

Conceptual resolver:

```cpp
EncoderState *resolveEncoderState(const char *name) {
  if (strcmp(name, "drive_front_left") == 0) {
    return &encoderDriveFrontLeft;
  }

  if (strcmp(name, "drive_front_right") == 0) {
    return &encoderDriveFrontRight;
  }

  return nullptr;
}
```

Conceptual command handler:

```cpp
void handleEncoderRead(const char *name) {
  EncoderState *state = resolveEncoderState(name);

  if (state == nullptr) {
    PI_SERIAL.print("ERR ENCODER unknown_name=");
    PI_SERIAL.println(name);
    return;
  }

  EncoderSnapshot snapshot = snapshotEncoder(*state);
  replyEncoder(name, snapshot);
}
```

Unknown names must return an error rather than silently reading arbitrary pins.

---

## 15. Initial `valid` and `valid_flags` Behaviour

The purpose of validity is initially diagnostic. The first implementation should not reject useful encoder data because of isolated irregularities.

### Available approaches

1. Always return valid and provide no diagnostic state.
2. Permanently invalidate an encoder after any invalid transition.
3. Start permissively and use only a minimal current-condition flag.
4. Build rate-based, timeout-based and noise-based validity logic immediately.

### Selected path for this robot

**Start with minimal permissive validity. `valid_flags` is diagnostic and should not implement rigorous control at this stage.**

Initial flag definitions:

```cpp
static const uint16_t ENC_FLAG_NONE            = 0x0000;
static const uint16_t ENC_FLAG_NOT_INITIALIZED = 0x0001;
```

Initial behaviour:

| Condition | `valid` | `valid_flags` |
|---|---:|---:|
| Encoder initialised | 1 | 0 |
| Encoder not initialised | 0 | 1 |

Normal response:

```text
valid=1 valid_flags=0
```

Before initialization:

```text
valid=0 valid_flags=1
```

### Invalid transition diagnostics

An invalid transition must increment:

```cpp
state.invalid_transition_count++;
```

It must not initially:

- permanently set `valid=0`;
- set a persistent fault bit;
- stop the encoder counter;
- stop robot motion.

The diagnostic counter can be exposed in a dedicated diagnostic command or serial debug output later. It is not one of the four initial Pi encoder IO properties.

Possible future flags include:

```cpp
ENC_FLAG_EXCESSIVE_INVALID_TRANSITIONS
ENC_FLAG_SAMPLING_OVERRUN
ENC_FLAG_SIGNAL_FAULT
ENC_FLAG_COUNT_OVERFLOW
```

These are deferred until physical testing shows a practical need.

---

## 16. Encoder Resolution Configuration

The current drive motors are DFRobot FIT0186 12 V geared motors with integrated quadrature encoders.

Manufacturer information identifies nominal values of:

| Attribute | Nominal value |
|---|---:|
| Gear ratio | 43.8:1 |
| Motor-shaft encoder count | 16 counts/revolution |
| Gearbox output count | approximately 700 counts/revolution |
| Nominal output speed | approximately 251 rpm |

Official references:

- DFRobot FIT0186 wiki: <https://wiki.dfrobot.com/fit0186/>
- Farnell UK product page: <https://uk.farnell.com/dfrobot/fit0186/dc-motor-251rpm-12vdc-1-77n-m/dp/3974138>

### Available configuration approaches

1. Configure only by generic encoder type.
2. Configure by complete motor, encoder and gearbox model.
3. Create a separate complete specification for every individual installed motor.

### Selected path for this robot

**Use a reusable encoder profile for each complete motor-and-gearbox model.**

Both drive encoders reference the same model profile:

```python
ENCODERS = {
    "drive_front_left": "dfrobot_fit0186",
    "drive_front_right": "dfrobot_fit0186",
}
```

Recommended profile concept:

```python
# config/encoders/dfrobot_fit0186.py

ENCODER_TYPE = "quadrature_hall"
COUNT_DECODING = "x4"

MOTOR_SHAFT_COUNTS_PER_REV_NOMINAL = 16
GEAR_RATIO_NOMINAL = 43.8
OUTPUT_SHAFT_COUNTS_PER_REV_NOMINAL = 700

# Set after physical verification.
OUTPUT_SHAFT_COUNTS_PER_REV = 700
```

The model profile contains manufacturer-level characteristics shared by all FIT0186 units.

Robot-specific installation details do not belong in the shared profile. They are configured separately by IO name.

---

## 17. Interpreting the Published Count

The published values are internally consistent:

\[
16 \times 43.8 = 700.8
\]

However, manufacturers do not always use the terms `count`, `pulse`, `cycle`, `CPR` and quadrature resolution consistently.

The firmware uses x4 transition decoding, but the published value must not automatically be multiplied by four without testing.

### Selected path for this robot

**Use 700 output-shaft counts per revolution as the nominal provisional value. Verify the actual count produced by the completed firmware over one gearbox-output revolution.**

Possible results:

| Observed count for one output revolution | Interpretation |
|---:|---|
| Approximately 700 | Published 700 value matches the firmware count convention |
| Approximately 1,400 | Published value may correspond to x2 relative to the firmware |
| Approximately 2,800 | Published value may represent cycles while x4 decoding counts four transitions per cycle |
| Other repeatable value | Use the observed value and investigate specification terminology |

The verified operational value should be stored directly:

```python
OUTPUT_SHAFT_COUNTS_PER_REV = <measured repeatable value>
```

Do not continuously derive the operational value at runtime from `16 × 43.8`. The measured gearbox-output count is clearer and accounts for the actual firmware counting convention.

---

## 18. Encoder Resolution Verification

### Manual one-revolution test

1. Raise the robot so the tested wheel can rotate freely.
2. Mark the wheel or gearbox output shaft and a fixed reference point.
3. Obtain the initial cumulative encoder count.
4. Rotate the wheel exactly one complete revolution by hand.
5. Obtain the final cumulative encoder count.
6. Calculate the absolute count difference.
7. Repeat at least five times.
8. Repeat in the opposite direction.
9. Compare the repeatability and sign.
10. Store the repeatable absolute count as `OUTPUT_SHAFT_COUNTS_PER_REV`.

### Powered-speed verification

After the manual test:

1. Run the wheel slowly for a fixed number of revolutions.
2. Record the resulting count.
3. Repeat at progressively higher motor speeds.
4. Confirm that counts per revolution remain stable.
5. If counts fall as speed increases, increase the timer sampling rate or investigate signal quality.

The initial goal is a stable, usable conversion value. Do not introduce complicated invalidation logic before basic counting is repeatable.

---

## 19. Encoder Sign Convention

Mirrored left and right motors may produce opposite raw count directions for robot-forward motion.

### Available approaches

1. Swap A and B wiring on one encoder.
2. Reverse direction in Arduino firmware.
3. Apply installation polarity in the Pi configuration.

### Selected path for this robot

**The Arduino reports raw electrically decoded direction. The Pi configuration applies robot-specific installation polarity.**

Conceptual robot configuration:

```python
ENCODER_SIGN = {
    "drive_front_left": 1,
    "drive_front_right": -1,
}
```

The actual values must be established during physical verification.

After applying configuration polarity:

```text
Positive count change = wheel moves the robot forward
Negative count change = wheel moves the robot backwards
```

Do not apply sign reversal in both firmware and Pi configuration.

---

## 20. Wheel and Drivetrain Calibration Location

### Available approaches

1. Hard-code wheel diameter and gearbox resolution in Arduino firmware.
2. Have the Arduino return millimetres instead of counts.
3. Return raw counts and keep calibration on the Pi.

### Selected path for this robot

**The Arduino returns raw cumulative x4 counts. Motor, encoder, wheel and installation calibration are resolved on the Pi.**

Relevant Pi-side values include:

```python
OUTPUT_SHAFT_COUNTS_PER_REV
EFFECTIVE_WHEEL_DIAMETER_MM
ENCODER_SIGN
EFFECTIVE_TRACK_WIDTH_MM
```

This keeps the Mega firmware hardware-oriented and reusable across robots.

---

## 21. Pi-Side Drive-Wheel Odometry

For consecutive samples:

\[
\Delta C_L = C_{L,new} - C_{L,old}
\]

\[
\Delta C_R = C_{R,new} - C_{R,old}
\]

After applying each encoder sign, convert count change to wheel travel:

\[
\Delta s_L = \Delta C_L \frac{\pi D_L}{N_L}
\]

\[
\Delta s_R = \Delta C_R \frac{\pi D_R}{N_R}
\]

where:

- `D` is effective wheel diameter;
- `N` is verified counts per wheel or gearbox-output revolution.

Centre displacement:

\[
\Delta s = \frac{\Delta s_R + \Delta s_L}{2}
\]

Heading change:

\[
\Delta \theta = \frac{\Delta s_R - \Delta s_L}{T}
\]

where `T` is effective track width.

Midpoint pose integration:

\[
x_{new} = x + \Delta s \cos\left(\theta + \frac{\Delta\theta}{2}\right)
\]

\[
y_{new} = y + \Delta s \sin\left(\theta + \frac{\Delta\theta}{2}\right)
\]

\[
\theta_{new} = \theta + \Delta\theta
\]

Drive-wheel odometry is useful for short-term motion measurement but is affected by:

- wheel slip;
- tyre compression;
- turning scrub;
- acceleration and braking;
- effective wheel diameter;
- effective track width;
- the rear omni-wheel geometry.

It should ultimately be fused with IMU and vision rather than treated as absolute localisation.

---

## 22. Suggested Firmware Definitions

The completed `.ino` file is expected to contain equivalents of:

```cpp
#include <util/atomic.h>

static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_A  = 22;
static const uint8_t PIN_ENC_DRIVE_FRONT_LEFT_B  = 24;
static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_A = 26;
static const uint8_t PIN_ENC_DRIVE_FRONT_RIGHT_B = 28;

static const uint16_t ENC_FLAG_NONE            = 0x0000;
static const uint16_t ENC_FLAG_NOT_INITIALIZED = 0x0001;

struct EncoderState {
  volatile int32_t count;
  volatile uint32_t last_edge_us;
  volatile uint16_t invalid_transition_count;
  volatile uint8_t previous_ab;
  volatile bool initialized;
};

struct EncoderSnapshot {
  int32_t count;
  uint16_t invalid_transition_count;
  bool initialized;
  uint32_t timestamp_ms;
};

EncoderState encoderDriveFrontLeft;
EncoderState encoderDriveFrontRight;

static const int8_t QUAD_TABLE[16] = {
   0, -1, +1,  0,
  +1,  0,  0, -1,
  -1,  0,  0, +1,
   0, +1, -1,  0
};
```

Function equivalents expected:

```cpp
uint8_t readLeftABFromPortA(uint8_t portA);
uint8_t readRightABFromPortA(uint8_t portA);

void updateEncoderFromAB(
    volatile EncoderState &state,
    uint8_t currentAB);

void setupEncoderSamplingTimer();

EncoderState *resolveEncoderState(const char *name);
EncoderSnapshot snapshotEncoder(const EncoderState &state);

void handleEncoderRead(const char *name);
void replyEncoder(
    const char *name,
    const EncoderSnapshot &snapshot);
```

The exact naming may follow the conventions already used in the `.ino` file.

---

## 23. Expected Firmware Setup Logic

The setup path should perform these steps in order:

1. Define encoder inputs with `INPUT_PULLUP`.
2. Read `PINA` once.
3. Initialise both `previous_ab` fields from the live pins.
4. Set both cumulative counts to zero.
5. Set both invalid-transition counters to zero.
6. Mark both encoder states initialized.
7. Configure and start the selected hardware timer.
8. Enable normal serial command processing.

The timer must not run before `previous_ab` is initialised.

---

## 24. Expected Firmware Read Logic

For every timer interrupt:

```text
read PINA once
    ↓
extract left A/B and right A/B
    ↓
compare each current state with its previous state
    ↓
unchanged: do nothing
valid one-bit transition: add +1 or -1
invalid two-bit transition: increment diagnostic counter
    ↓
store current state as previous state
```

For every semantic serial read:

```text
ENCODER <name> READ
    ↓
resolve semantic name
    ↓
copy the complete encoder state atomically
    ↓
set snapshot timestamp
    ↓
calculate minimal valid and valid_flags
    ↓
return one response containing all public fields
```

---

## 25. Expected Serial Responses

### Successful left encoder read

```text
OK ENCODER drive_front_left count=12345 timestamp_ms=482761 valid=1 valid_flags=0
```

### Successful right encoder read

```text
OK ENCODER drive_front_right count=-12340 timestamp_ms=482764 valid=1 valid_flags=0
```

Raw left and right signs may differ before Pi configuration applies installation polarity.

### Encoder not initialized

```text
OK ENCODER drive_front_left count=0 timestamp_ms=100 valid=0 valid_flags=1
```

### Unknown encoder name

The exact error syntax should follow the existing serial protocol conventions. A reasonable form is:

```text
ERR ENCODER unknown_name=drive_rear_left
```

---

## 26. Implementation Sequence

Complete the `.ino` file in this order:

1. Add `EncoderState` and `EncoderSnapshot`.
2. Add minimal validity flag definitions.
3. Add the x4 transition table.
4. Add direct `PINA` extraction helpers.
5. Add the encoder transition update function.
6. Inventory hardware timer use in the complete firmware.
7. Select and configure a free timer.
8. Initialise both live A/B states before starting the timer.
9. Add the timer interrupt handler.
10. Add semantic name resolution.
11. Add atomic snapshot logic.
12. Add `ENCODER <name> READ` parsing.
13. Add the formatted response.
14. Compile and resolve timer/library conflicts.
15. Test raw A/B states with `READ QUAD` if needed.
16. Rotate each wheel manually and verify count direction.
17. Measure counts per complete output-shaft revolution.
18. Test repeatability at powered speeds.
19. Configure Pi encoder profiles and installation polarity.
20. Only then implement or enable Pi-side differential-drive odometry.

---

## 27. Verification

### A. Firmware compilation

Verify:

- no timer conflicts;
- no duplicate ISR definitions;
- no type or volatility errors;
- serial command parsing still works;
- existing motors, servos and receiver functions still compile.

### B. Stationary test

With both wheels stationary:

- repeated reads return unchanged counts;
- `timestamp_ms` continues to advance;
- `valid=1` after initialization;
- `valid_flags=0` after initialization.

### C. Manual direction test

For each wheel independently:

1. note the initial count;
2. rotate the wheel forward;
3. confirm count changes consistently;
4. rotate backward;
5. confirm the count reverses;
6. record the Pi-side polarity needed for positive-forward semantics.

### D. One-revolution resolution test

Perform the procedure in Section 18 and set:

```python
OUTPUT_SHAFT_COUNTS_PER_REV
```

from the measured repeatable result.

### E. Speed test

At several motor powers:

- confirm count per revolution remains stable;
- confirm both encoders behave similarly;
- review invalid-transition counts diagnostically;
- increase sampling rate only if testing indicates missed transitions.

### F. Serial contract test

Verify that each request returns exactly one parseable response containing:

```text
count=
timestamp_ms=
valid=
valid_flags=
```

### G. Pi IO map test

Confirm one response populates all four properties for the addressed encoder:

```python
io.encoder[name].count
io.encoder[name].timestamp_ms
io.encoder[name].valid
io.encoder[name].valid_flags
```

---

## 28. Initial Acceptance Criteria

The initial encoder implementation is acceptable when:

- both encoders count continuously without depending on the normal loop rate;
- counts increase and decrease consistently with wheel direction;
- stationary counts remain stable;
- one-revolution counts are repeatable;
- counts remain reasonably stable across the expected speed range;
- one semantic serial request returns all four required properties;
- the Pi can parse both left and right responses;
- invalid transitions are recorded diagnostically but do not unnecessarily suppress useful data;
- the exact FIT0186 operational count is stored in the Pi model profile;
- installation polarity is stored on the Pi, not duplicated in the firmware.

The initial implementation does not require:

- automatic stall detection;
- automatic signal-fault detection;
- strict invalid-transition thresholds;
- encoder-based motion shutdown;
- speed estimation from `last_edge_us`;
- a last-edge Pi IO property;
- complete pose calculation on the Arduino.

---

## 29. Selected Architecture Summary

| Decision | Selected path |
|---|---|
| Encoder processing location | Arduino decodes and counts; Pi performs odometry |
| Serial access | One requested response per encoder containing four fields |
| Public timestamp | Arduino snapshot time |
| Decode resolution | x4 |
| Decode method | Previous/current transition lookup table |
| Mega pins | 22, 24, 26 and 28 retained |
| Acquisition method | Hardware timer polling |
| Pin read | One direct `PINA` read per timer interrupt |
| Count storage | Signed cumulative `int32_t` |
| Serial snapshot | Short atomic copy, then print with interrupts enabled |
| Validity | Minimal and permissive; not-initialized only initially |
| Invalid transitions | Diagnostic counter; no automatic invalidation initially |
| Motor/encoder configuration | Reusable complete model profile: `dfrobot_fit0186` |
| Nominal output count | 700 counts/revolution, provisional until measured |
| Installation polarity | Pi configuration by encoder IO name |
| Wheel and track calibration | Pi configuration/calibration |
| `READ QUAD` | Low-level wiring diagnostic only |
| Design priority | Obtain stable workable values before rigorous fault control |

---

## 30. Next Step

Use this document to update the Mega `.ino` file through the encoder implementation sequence in Section 26.

The immediate next coding task is:

1. inspect the complete firmware for timer use;
2. select the encoder sampling timer;
3. add the encoder state, direct-port decoder and timer ISR;
4. add `ENCODER drive_front_left READ` and `ENCODER drive_front_right READ`;
5. compile and perform manual count verification.
