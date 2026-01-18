# hw_io — Hardware IO Boundary (Primary)

This folder defines the **primary hardware boundary layer** for the robot.

It exposes a single canonical interface, `IOMap`, which presents robot hardware as
**capabilities** (motors, servos, cameras, sensors, digital outputs) in a robot-agnostic way.

No behaviours, strategy, or motion planning belongs here.

---

## Design Goals

- One behavioural codebase runs on:
  - SR simulator
  - SR hardware
  - SR-equivalent hardware without `sr.robot3`
  - custom robots with different wiring
- Remove conflation between:
  - simulation vs real
  - robot identity vs hardware wiring
  - calibration vs IO construction
- Make IO selection explicit via `hardware_profile` (not `robot_id`)

---

## Architecture

Behaviours / Primitives
↓
Controller (constructs io, level2, motion backend)
↓
Level2 (robot-agnostic actions)
↓
hw_io (IOMap implementation chosen by hardware_profile)
↓
SR Robot3 / Arduino / GPIO / external devices

---

## Core Interface: `IOMap`

`hw_io/base.py` defines:

- Sensors:
  - `bumpers() -> Dict[str, bool]`
  - `reflectance() -> Dict[str, float]`
  - `ultrasonics() -> Dict[str, Optional[float]]`
  - `sense() -> Dict[str, Any]` (snapshot convenience)

- Cameras:
  - `cameras() -> Dict[str, CameraLike]` keyed by semantic name (`"front"`, `"rear"`)

- Actuators:
  - `motors` (exposed motor objects, not controlled here)
  - `servos`

- Digital Outputs:
  - `outputs -> DigitalOutputs | None`
    - named outputs such as `"VACUUM"` / `"SOLENOID"` / `"RELAY_*"`
    - `set(name, on)` / `get(name)` / `names()`

- Power:
  - `battery() -> Dict[str, Optional[float]]`

- Timing:
  - `sleep(secs)` (uses robot time when available)

---

## Selecting an IOMap

`hw_io/resolve.py` should select an IOMap implementation based **only** on:

- `hardware_profile` (e.g. `"sr1"`)

Example shape:

```py
def resolve_io(*, robot, hardware_profile: str) -> IOMap:
    if hardware_profile == "sr1":
        return SR1IO(robot)
    raise ValueError(...)

Implementations
hw_io/sr1.py

SR1 IOMap implementation.

Typically binds to:

robot.motor_board.motors

robot.servo_board.servos

robot.arduino (for bumpers, reflectance, ultrasonics)

robot.power_board.outputs[...] (for named digital outputs like VACUUM / SOLENOID)

robot.camera (AprilCamera)

Important:

This is the ONLY place SR APIs and pin numbers should appear.

Cameras

Camera wrappers live under hw_io/cameras/ so the rest of the stack can treat cameras uniformly
(e.g. SRAprilCamera wraps SR’s AprilCamera and exposes a stable .see()-like API).

Conventions

Semantic naming:

cameras: "front", "rear", etc.

outputs: "VACUUM" (or "SOLENOID", but pick one and keep it stable)

sensors: "fl", "fr", "rl", "rr", etc.

No behaviour:

hw_io must not contain motion logic, state machines, or strategy.

Roadmap / SR2 Note

SR2 is expected to share concepts:

Arduino + digital outputs remain conceptually similar

motors/servos may require:

a different driver, or

mapping to board pins via a separate implementation

This becomes just another IOMap implementation selected via hardware_profile.