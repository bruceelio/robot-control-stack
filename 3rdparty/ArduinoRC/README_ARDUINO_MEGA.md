
# Raspberry Pi ↔ Arduino Mega Link

This document defines the control link between the Raspberry Pi and the Arduino Mega.

Its purpose is to make the control architecture explicit, especially the distinction between:

- **semantic robot control on the Pi**
- **hardware execution on the Arduino Mega**

This is a custom serial protocol over USB. It is intentionally simple, human-readable, and easy to test before integrating into `hw_io`.

---

## 1. High-Level Architecture

The robot has two potential control sources:

- **FlySky receiver** → local teleop input
- **Raspberry Pi** → autonomous / higher-level input

The Arduino Mega is the hardware controller. It owns:

- motor output
- servo output
- FlySky iBus input
- arbitration between Pi and FlySky

The Raspberry Pi does **not** directly toggle Arduino pins or send PWM.

Instead, the Pi sends **text commands** over the USB serial link, and the Arduino interprets them.

---

## 2. Key Design Principle

### The Pi owns the meaning.
### The Arduino owns the execution.

That means:

- On the Pi side, code thinks in terms of:
  - motor power
  - servo position
  - autonomy vs teleop
- On the Arduino side, code thinks in terms of:
  - parsed serial commands
  - motor driver output
  - servo pulse generation
  - heartbeat timeout

---

## 3. What the Serial Link Is

The Pi ↔ Mega link is a **newline-delimited ASCII command protocol** over USB serial.

Each line is a command.

Examples:

```text
HELLO
MODE AUTO
HB 1
DRV 0.25 0.25
GRIP -1.0
STOP
MODE TELEOP
````

This is **not**:

* JSON
* a dictionary
* shared variables
* a structured object transport

It is simply a stream of text commands.

---

## 4. Important Concept: Command Protocol, Not Shared State

A common mental model is to imagine the Pi sending something like:

```python
mode = "AUTO"
io.motors.left = 0.25
```

That is **not** how this link works.

Instead, the Pi sends imperative commands like:

```text
MODE AUTO
DRV 0.25 0.25
```

The Arduino is effectively acting as a **small command interpreter**.

It reads one line at a time and decides what to do based on the exact command text.

---

## 5. How the Arduino Understands Commands

The Arduino reads bytes from USB serial (`Serial`) until it sees a newline.

For example, the Pi sends:

```text
MODE AUTO\n
```

The Arduino builds the string:

```text
MODE AUTO
```

and passes it to its command handler.

That handler contains logic like:

```cpp
if (strcmp(line, "MODE AUTO") == 0) {
    piAutoRequested = true;
}
```

So the meaning comes from the Arduino code itself.

There is no special built-in USB meaning for `"MODE AUTO"`.

---

## 6. Control Arbitration: Pi vs FlySky

The Arduino supports **two input paths**:

* **FlySky**
* **Pi**

The Arduino decides which one to use in real time.

### FlySky path

This is the default path.

If the Pi is absent, inactive, or not in AUTO mode, the Arduino uses FlySky input.

### Pi path

The Pi only takes control when:

1. it explicitly requests AUTO mode
2. it keeps sending heartbeats

If either of those is missing, control returns to FlySky.

---

## 7. How Pi Takes Control

The Pi sends:

```text
MODE AUTO
```

This does **not** directly move motors or servos.

It only tells the Arduino:

> the Pi is requesting authority

The Pi must also keep sending heartbeats:

```text
HB 1
HB 2
HB 3
...
```

The Arduino only stays under Pi control while the heartbeat remains fresh.

---

## 8. Heartbeat Logic

The heartbeat exists so the Arduino can safely detect when the Pi has crashed, disconnected, or stopped responding.

### While heartbeat is fresh:

* Pi keeps control (if AUTO was requested)

### If heartbeat times out:

* Arduino immediately drops back to FlySky control
* motors are stopped during the transition

This is the core failsafe behavior.

---

## 9. Practical Control Decision

Conceptually, the Arduino loop does this:

```text
if Pi requested AUTO and heartbeat is fresh:
    use Pi commands
else:
    use FlySky commands
```

So the Pi is **not permanently taking over**.

It is only allowed to control the robot while it is actively proving it is alive.

---

## 10. Supported Commands

### `HELLO`

Request board identity.

**Pi sends:**

```text
HELLO
```

**Arduino replies:**

```text
ID MEGA_AUX_1
```

Used to confirm that the correct board is connected.

---

### `MODE AUTO`

Request Pi control.

**Pi sends:**

```text
MODE AUTO
```

**Arduino replies:**

```text
OK MODE AUTO
```

This sets an internal flag that the Pi wants control.

Actual control only happens if heartbeats are also fresh.

---

### `MODE TELEOP`

Release Pi control and return to FlySky.

**Pi sends:**

```text
MODE TELEOP
```

**Arduino replies:**

```text
OK MODE TELEOP
```

This explicitly returns control to the FlySky path.

---

### `HB <seq>`

Heartbeat.

**Pi sends:**

```text
HB 1
HB 2
HB 3
```

**Arduino replies:**

```text
OK HB 1
OK HB 2
OK HB 3
```

The sequence number is not strictly required for control, but it is useful for debugging.

---

### `DRV <left> <right>`

Set direct motor power.

Example:

```text
DRV 0.25 0.25
DRV -0.30 0.30
```

Range:

* `-1.0` = full reverse
* `0.0` = stop
* `1.0` = full forward

These values are **normalized power commands**, not raw PWM values.

The Arduino converts them into RoboClaw output commands.

---

### `GRIP <pos>`

Set gripper position.

Example:

```text
GRIP -1.0
GRIP 0.0
GRIP 1.0
```

Range:

* `-1.0` = fully open
* `0.0` = midpoint
* `1.0` = fully closed

The Arduino converts this normalized position into servo pulse widths.

---

### `STOP`

Immediately stop motors.

**Pi sends:**

```text
STOP
```

**Arduino replies:**

```text
OK STOP
```

This does not necessarily exit AUTO mode. It only commands motor stop.

---

## 11. Why This Uses Plain Text

The protocol is plain text on purpose.

Advantages:

* easy to test manually in a serial terminal
* easy to debug by inspection
* easy to log
* lightweight for Arduino
* no JSON parsing or external libraries needed

This is a good fit for embedded control.

---

## 12. What This Protocol Is Trying to Preserve

The longer-term goal is to be compatible with a **Student Robotics style control model**.

That means the Pi should think in terms of:

* normalized motor power
* normalized servo position
* semantic outputs

So the Mega protocol uses commands like:

* `DRV left right`
* `GRIP pos`

rather than raw PWM duty cycles or raw pin toggles.

This makes it easier to later replace Mega-owned output behavior with SR boards while keeping the Pi-side semantics similar.

---

## 13. What the Pi Is Responsible For

The Pi is responsible for:

* deciding whether autonomy is active
* sending `MODE AUTO`
* sending regular heartbeat messages
* sending normalized motor/servo commands
* explicitly returning to `MODE TELEOP` when appropriate

The Pi is **not** responsible for:

* generating PWM
* directly driving servo pulses
* directly driving motor pins
* performing low-level IO timing

---

## 14. What the Arduino Mega Is Responsible For

The Arduino Mega is responsible for:

* parsing incoming serial commands
* deciding whether Pi or FlySky currently owns control
* enforcing heartbeat timeout behavior
* generating actual motor/servo outputs
* keeping local teleop operational even when the Pi is absent

---

## 15. Default Behavior

### No Pi connected

* Arduino runs FlySky teleop normally

### Pi connected but silent

* Arduino still runs FlySky teleop normally

### Pi connected and sends `MODE AUTO` + heartbeat

* Arduino defers to Pi commands

### Pi stops sending heartbeat

* Arduino drops back to FlySky automatically

This makes the system robust when the Pi is sometimes connected and sometimes not.

---

## 16. Example Session

A typical Pi session might look like this:

```text
HELLO
MODE AUTO
HB 1
DRV 0.20 0.20
HB 2
GRIP -1.0
HB 3
GRIP 1.0
STOP
MODE TELEOP
```

Meaning:

1. identify the board
2. request Pi control
3. keep control alive
4. drive motors
5. move gripper
6. stop motors
7. return control to FlySky

---

## 17. Why This Is Being Tested First

The serial link and Arduino behavior are being tested before integrating into `hw_io` so that later failures can be isolated cleanly.

This stage is intended to prove:

* USB serial link works
* command parser works
* heartbeat timeout works
* AUTO/TELEOP arbitration works
* motor/servo commands work
* hardware responds correctly

Only after this is stable should the Pi-side `hw_io` abstraction be layered on top.

---

## 18. Relationship to `hw_io`

Later, `hw_io` will translate semantic robot control into these text commands.

For example:

```python
io.motors[0].power = 0.25
io.servos[0].position = -1.0
```

might later become:

```text
DRV 0.25 0.25
GRIP -1.0
```

So `hw_io` will be the translation layer between:

* Pi-side semantic robot code
* Mega serial command protocol

At this stage, the protocol is being tested directly, without `hw_io`, to reduce complexity.

---

## 19. Device Identity

The Arduino exposes a simple textual identity, e.g.:

```text
ID MEGA_AUX_1
```

This is useful for debugging and confirming that the correct board is connected.

This is separate from Linux serial device paths such as:

```text
/dev/ttyACM0
```

The textual identity is part of the protocol.
The Linux device path is assigned by the operating system.

---

## 20. Summary

* Pi and Mega communicate over **USB serial**
* The protocol is **plain text, line-based**
* The Arduino is a **command interpreter**
* The Pi must request AUTO and maintain heartbeat to control the robot
* If heartbeat expires, control falls back to FlySky
* Commands are semantic (`DRV`, `GRIP`), not raw PWM
* This protocol is being tested directly before integrating into `hw_io`

---

## 21. Next Step

The next step is to create hardware-in-the-loop tests inside the Pi test framework to validate:

* board handshake
* mode switching
* heartbeat timeout
* drive commands
* grip commands

before building the Pi-side `hw_io` adapter layer.
