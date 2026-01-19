# hw_io — Hardware IO Boundary (Primary)

This folder defines the **primary hardware boundary layer** for the robot.

It exposes a single robot-agnostic interface, `IOMap`, which presents robot hardware as
**capabilities** (motors, servos, cameras, sensors, digital outputs) behind stable method names.

No behaviours, strategy, or motion planning belongs here.

---

## Design Goals

- One behavioural codebase runs on:
  - SR simulator
  - SR hardware
  - SR-equivalent hardware without `sr.robot3`
  - custom robots with different wiring
- Make IO selection explicit via `hardware_profile` (not `robot_id`)
- Keep **all SR APIs and pin numbers** inside this folder only

---

## Architecture (Control Path)

Behaviours / Primitives  
↓  
Controller (constructs IOMap, Level2, Motion Backend)  
↓  
Level2 (robot-agnostic actions)  
↓  
hw_io (IOMap implementation chosen by `hardware_profile`)  
↓  
SR Robot3 / Arduino / GPIO / external devices

---

## Core Interface: `IOMap`

`hw_io/base.py` defines capabilities such as:

### Sensors
- `bumpers() -> Dict[str, bool]`
- `reflectance() -> Dict[str, float]`
- `ultrasonics() -> Dict[str, Optional[float]]`
- `sense() -> Dict[str, Any]` (snapshot convenience)

### Cameras
- `cameras() -> Dict[str, CameraLike]`
  - keyed by semantic name (`"front"`, `"rear"`, ...)

### Actuators
- `motors` (exposes motor objects; Level2 controls motion)
- `servos`

### Digital Outputs
- `outputs`
  - named outputs such as `"VACUUM"`
  - `set(name, on)` / `get(name)` / `names()`

### Power / System
- `battery() -> Dict[str, Optional[float]]`
- `sleep(secs)` (uses robot time when available)

Optional capabilities (may be `None` depending on robot):
- `kch()` (Brain Board LEDs)
- `buzzer()` (PowerBoard piezo)
- `wait_start()` (SR start gate)

---

## Selecting an IOMap

`hw_io/resolve.py` selects an IOMap implementation based only on:

- `hardware_profile` (e.g. `"sr1"`)

Example:

```py
def resolve_io(*, robot, hardware_profile: str) -> IOMap:
    if hardware_profile == "sr1":
        return SR1IO(robot)
    raise ValueError(f"Unknown hardware_profile: {hardware_profile}")

Implementations:

hw_io/sr1.py — SR1 mapping

SR1IO (hw_io/sr1.py)

SR1 binds to SR Robot3 boards when present:

Motors: robot.motor_board.motors

Servos: robot.servo_board.servos

Arduino sensors:

bumpers (D10–D13)

reflectance (A0–A2)

ultrasonics (trig/echo pairs)

PowerBoard outputs:

"VACUUM" -> OUT_H0

Camera:

"front" -> SR AprilCamera wrapper

IMPORTANT:
This is the ONLY place SR APIs and pin numbers should appear.

Arduino API compatibility

SR Arduino method names can vary between simulator and real firmware versions.
SR1IO uses internal compatibility helpers to resolve the correct read functions.

Cameras

Camera wrappers live under hw_io/cameras/ so the rest of the stack can treat cameras uniformly.

Example:

SRAprilCamera wraps SR’s AprilCamera and exposes a stable .see() API.

Conventions

Semantic naming:

cameras: "front", "rear", ...

outputs: "VACUUM" (pick one canonical name and keep it stable)

bumpers: "fl", "fr", "rl", "rr"

hw_io must not contain:

motion logic

behaviours / state machines

strategy / targeting

Testing / IO Checkout

IO checkout tests live under tests/test_io_checkout.py and run in RunMode.TESTS.

Read-only tests are enabled by default.

Actuation tests (motors/vacuum/buzzer/LED) should be opt-in.

To run:

set RUN_MODE = RunMode.TESTS in config/strategy.py

run the robot program as normal

the controller will dispatch to run_tests(robot=robot)


---

## 4) Why your KCH log prints tuples
Your log shows:

`KCH LED_A -> (True, False, False)`

That’s not wrong — it’s the `Colour` enum being printed in its tuple form in that SR version. Totally fine.

---

If you want, paste your current `tests/test_io_checkout.py` and I’ll tune the tests so:
- actuator tests are **disabled by default**
- `buzzer.off()` doesn’t mark PASS when it actually failed (right now you PASS even after the error; we should treat off-failure as FAIL unless you explicitly mark it “known issue”).

## IO Checkout (SR1 Sim / SR1 Real)

IO checkout tests live in `tests/test_io_checkout.py` (category: `io`).

Run them by setting:

- `config/strategy.py`: `RUN_MODE = RunMode.TESTS`

Then run the robot program normally.
The controller will dispatch to:

- `tests/runner.py::run_tests(robot=robot, category="io")`

### Safety
Actuation tests (motors/vacuum/buzzer/LED) should be `enabled=False` by default.
Enable them temporarily only during bring-up.
