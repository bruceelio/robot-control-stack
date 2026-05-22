motion_backends/README_MOTION_BACKENDS.md

# Motion Backend Package

This directory contains the robot-agnostic motion backend layer. Its job is to convert high-level motion requests into normalized canonical motor power commands, without knowing anything about physical motor pins, motor drivers, CAN devices, PWM channels, or hardware-specific output modes.

The motion backend layer sits between command generation/path tracking and the lower-level motor output pipeline.

```text
Command Generators / Path Tracking
        ↓
Motion Backends
        ↓
motion_mux.py
        ↓
Motor Output Conditioner
        ↓
Level2 / HAL / IOMap
        ↓
Motor Board / Physical Motors
```

## Design Goal

The main goal of this package is to keep robot motion logic hardware-agnostic.

A robot may be:

- 2WD differential drive
- 4WD tank / skid-steer drive
- 4WD mecanum / holonomic drive
- Open-loop timed drive/rotate
- Closed-loop velocity controlled

The rest of the robot framework should not need to know the details of how each drivetrain is controlled. Each backend receives canonical motion inputs and produces normalized motor power outputs.

## Important Boundary Rules

### Motion backends do not talk to hardware

Motion backends should not know about:

- GPIO pins
- PWM channels
- CAN IDs
- motor board models
- voltage mode
- torque mode
- motor driver protocol

That belongs to the HAL, IOMap, MotorBoard, or other lower-level hardware interface.

### Motion backends output normalized motor power

Backend outputs should be normalized effort commands, typically in this range:

```text
-1.0 to +1.0
```

These are not physical units. They are canonical motor power requests.

### Motor conditioning happens after backend selection

Backends produce raw motor requests. The shared motor output conditioner applies:

- e-stop / motion inhibit
- normalization
- deadband
- slew/ramp limiting
- clamping to maximum power
- motor polarity

This prevents every backend from duplicating the same output safety logic.

---

# Package Contents

```text
motion_backend/
│
├── __init__.py
├── motion_mux.py
│
├── timed_drive.py
├── timed_rotate.py
│
├── vel_diff_2wd.py
├── vel_tank_4wd.py
└── vel_mecanum.py
```

---

# `__init__.py`

## Purpose

Package initialization and module exposure.

This file should expose the public backend classes/functions used by the rest of the robot framework.

## Typical responsibility

```python
from .motion_mux import MotionMux
from .timed_drive import TimedDrive
from .timed_rotate import TimedRotate
from .vel_diff_2wd import VelocityDiff2WD
from .vel_tank_4wd import VelocityTank4WD
from .vel_mecanum import VelocityMecanum
```

This allows other code to import motion backends cleanly:

```python
from motion_backend import MotionMux, VelocityMecanum
```

---

# `motion_mux.py`

## Purpose

Backend selector and data router.

The motion mux decides which backend currently owns drivetrain output authority.

It receives candidate outputs from available motion backends and routes one selected backend to canonical `mux_*` outputs.

## Why this exists

Only one motion backend should control the drivetrain at a time.

For example, the robot should not apply open-loop drive, closed-loop mecanum, and tank steering outputs simultaneously. The mux provides a clean arbitration point.

## Inputs

Typical backend output groups:

```text
drv_*    from timed_drive.py
rot_*    from timed_rotate.py
diff_*   from vel_diff_2wd.py
tnk_*    from vel_tank_4wd.py
mec_*    from vel_mecanum.py
```

## Outputs

Canonical mux outputs:

```text
mux_motor_power_front_left
mux_motor_power_front_right
mux_motor_power_rear_left
mux_motor_power_rear_right
```

For 2WD systems, rear outputs may be zero, disabled, or unused depending on the motor output conditioner and motor configuration.

## Typical selection input

```text
motor_backend_sel
```

Possible selections may include:

```text
timed_drive
timed_rotate
velocity_diff_2wd
velocity_tank_4wd
velocity_mecanum
```

## When to use

Use the mux whenever multiple motion backends exist in the system and only one should be allowed to command motor output.

## Important behavior

The mux should be deterministic. If no valid backend is selected, it should output safe zero motor commands.

---

# `timed_drive.py`

## Purpose

Open-loop distance tracking primitive.

This backend is intended for simple programmed forward/reverse movement where the robot drives for a calculated or configured time.

## Output prefix

```text
drv_*
```

Example outputs:

```text
drv_motor_power_front_left
drv_motor_power_front_right
drv_motor_power_rear_left
drv_motor_power_rear_right
```

For a 2WD robot, only front-left and front-right may be meaningful.

## Typical inputs

```text
drive_factor
voltage_compensation
duration_s
cmd_drive_distance_mm
```

## How it works

The backend estimates the power and time required to perform a forward or reverse drive command. It does not rely on closed-loop velocity feedback.

Conceptually:

```text
requested distance
        ↓
drive scaling / calibration
        ↓
timed motor power request
        ↓
drv_* outputs
```

## When to use

Use this backend for:

- simple scripted movement
- low-cost robots without reliable feedback
- early bring-up/testing
- educational robots
- simple drive primitives where high precision is not required

## When not to use

Do not use this backend when accurate autonomous positioning is required.

Open-loop timed movement is sensitive to:

- battery voltage
- surface friction
- carpet/tile changes
- wheel slip
- robot weight
- motor variation
- starting/stopping dynamics

## Notes

`drive_factor` is an open-loop calibration value. It is not a replacement for localization or closed-loop velocity control.

# Timed Drive Design Considerations

## 1. Open-Loop Motion Is Not Position Control

`timed_drive.py` estimates motion from power and time. It does not know whether the robot actually reached the requested distance. Battery voltage, surface friction, wheel slip, load, and motor variation all affect the final position.

## 2. Drive Factor Is a Calibration Term

`drive_factor` compensates for predictable differences between commanded and observed travel distance. It should be treated as a surface/load calibration value, not as localization.

## 3. Voltage Compensation Helps but Does Not Close the Loop

`voltage_compensation` can reduce drift caused by battery sag, but it cannot correct wheel slip, collisions, or changing floor conditions.

## 4. Best Use Case

This backend is best for simple robots, basic scripted movement, bring-up testing, and low-precision motion primitives.

---

# `timed_rotate.py`

## Purpose

Open-loop angular rotation primitive.

This backend is intended for simple programmed robot rotation where the robot turns for a calculated or configured time.

## Output prefix

```text
rot_*
```

Example outputs:

```text
rot_motor_power_front_left
rot_motor_power_front_right
rot_motor_power_rear_left
rot_motor_power_rear_right
```

## Typical inputs

```text
rotate_factor
voltage_compensation
duration_s
cmd_rotate_angle_deg
```

## How it works

The backend estimates the power and time required to rotate the robot by a requested angle.

Conceptually:

```text
requested angle
        ↓
rotation scaling / calibration
        ↓
timed motor power request
        ↓
rot_* outputs
```

## When to use

Use this backend for:

- simple scripted turns
- testing motor polarity
- basic robot movement demos
- robots without usable heading feedback

## When not to use

Do not use this backend when accurate heading control is required.

Open-loop rotation is especially sensitive to:

- wheel slip
- surface friction
- battery voltage
- robot center of rotation
- drivetrain asymmetry

## Notes

`rotate_factor` is an open-loop calibration value. For better rotation accuracy, use a closed-loop backend with `robot_angular_z_rps` feedback from localization/source IO.

# Timed Rotate Design Considerations

## 1. Open-Loop Rotation Is Highly Surface Dependent

Timed rotation is usually less repeatable than timed straight driving because turning depends heavily on wheel scrub, friction, and drivetrain symmetry.

## 2. Rotate Factor Is Empirical

`rotate_factor` should be tuned by comparing requested rotation angle against observed rotation angle. It is not a substitute for IMU or localization feedback.

## 3. Overshoot and Static Friction

Small turns may fail to overcome static friction, while larger turns may overshoot once the robot begins rotating. Conservative slew limits can help reduce harsh starts and stops.

## 4. Best Use Case

This backend is useful for simple demonstrations, drivetrain testing, and robots without reliable heading feedback.

---

# `vel_diff_2wd.py`

## Purpose

Closed-loop 2WD differential drive velocity tracking.

This backend is for robots with two driven sides: left and right. It controls forward/reverse velocity and rotation rate using body-space velocity feedback.

## Output prefix

```text
diff_*
```

Example outputs:

```text
diff_motor_power_front_left
diff_motor_power_front_right
```

Rear outputs are normally unused for true 2WD differential drive.

## Typical command inputs

```text
nonmec_cmd_vel_linear_x_mps
nonmec_cmd_vel_angular_z_rps
```

## Typical feedback inputs

```text
diff_robot_linear_x_mps
diff_robot_angular_z_rps
```

These feedback signals come from localization/source IO. The backend should not care whether the estimate came from drive encoders, dead wheels, IMU, vision, or sensor fusion.

## Typical configuration

```text
diff_wheel_base_2wd_m
diff_wheel_radius_2wd_m
diff_pid_gains_linear_x
diff_pid_gains_angular_z
diff_velocity_limit_linear_x_mps
diff_velocity_limit_angular_z_rps
vel_watchdog_timeout_ms
```

## How it works

The backend compares commanded body velocity against measured body velocity:

```text
linear_x_error  = cmd_vel_linear_x_mps  - robot_linear_x_mps
angular_z_error = cmd_vel_angular_z_rps - robot_angular_z_rps
```

The controller converts those errors into corrected robot motion effort, then maps that effort into left/right motor power.

## When to use

Use this backend for:

- 2WD differential robots
- closed-loop autonomous driving
- path tracking without lateral strafe
- robots where localization provides usable linear and angular velocity estimates

## When not to use

Do not use this backend for mecanum or holonomic robots that need lateral Y motion.

# Velocity Differential 2WD Design Considerations

## 1. Natural Turning Behavior

A 2WD differential robot with casters usually turns more predictably than a 4WD skid-steer because only the driven wheels define the rotation and the caster wheels do not resist lateral motion as strongly.

## 2. Body-Velocity Feedback

This backend should control `robot_linear_x_mps` and `robot_angular_z_rps`, not individual wheel encoder velocity directly. The source of those velocity estimates belongs to localization/source IO.

## 3. Wheel Base Accuracy Matters

`diff_wheel_base_2wd_m` affects the relationship between forward velocity, angular velocity, and left/right motor effort. Incorrect wheelbase values will cause rotation tracking errors.

## 4. No Lateral Motion

This backend cannot command `lateral_y`. Any path tracking algorithm feeding this backend must convert path error into forward velocity and heading correction only.

---

# `vel_tank_4wd.py`

## Purpose

Closed-loop 4WD tank / skid-steer velocity tracking.

This backend is for robots with four driven wheels where the left side and right side act as paired drive groups.

It is similar to differential drive, but produces four motor outputs.

## Output prefix

```text
tnk_*
```

Example outputs:

```text
tnk_motor_power_front_left
tnk_motor_power_front_right
tnk_motor_power_rear_left
tnk_motor_power_rear_right
```

## Typical command inputs

```text
nonmec_cmd_vel_linear_x_mps
nonmec_cmd_vel_angular_z_rps
```

## Typical feedback inputs

```text
tank_robot_linear_x_mps
tank_robot_angular_z_rps
```

## Typical configuration

```text
tank_eff_wheel_base_4wd_m
tank_wheel_track_4wd_m
tank_wheel_radius_4wd_m
tank_pid_gains_linear_x
tank_pid_gains_angular_z
tank_velocity_limit_linear_x_mps
tank_velocity_limit_angular_z_rps
vel_watchdog_timeout_ms
```

## How it works

The backend performs closed-loop body velocity control for:

```text
linear_x
angular_z
```

It does not command lateral Y motion.

The output is distributed to the four drive motors:

```text
left side  → front_left, rear_left
right side → front_right, rear_right
```

## When to use

Use this backend for:

- 4WD skid-steer robots
- tank-drive robots
- non-mecanum 4WD robots
- robots where left and right wheel groups control forward/reverse and rotation

## When not to use

Do not use this backend when the drivetrain can intentionally strafe. Use `vel_mecanum.py` for holonomic movement.

## Notes

For skid-steer robots, the effective wheelbase/turning geometry may not perfectly equal the physical wheelbase because turning includes scrub and slip. That is why `tank_eff_wheel_base_4wd_m` may be more useful than a purely physical measurement.

## Design Considerations

The primary physical, mathematical and operational issues unique to a 4WD Tank/Skid-Steer Motion Backend stem from mechanical scrubbing resistance.
Because standard high-traction rubber wheels cannot slide laterally or steer mechanically, a 4WD tank drive is forced to intentionally lose traction and "skid" sideways across the ground in order to rotate. This introduces distinct architectural challenges:
## 1. High Tuning Gains (pid_gains_angular_z)
Dragging four high-traction rubber wheels sideways requires significantly more raw torque than turning a 2WD caster platform or sliding on Mecanum rollers. Because of this massive lateral friction, your angular turning PID values will need to be tuned much more aggressively. A low tuning gain will cause the robot to completely stall when commanded to turn in place, while an unoptimized high gain can cause the robot to violently overshoot once it breaks static friction.
## 2. Slew Rate and Watchdog Tuning
Sudden torque spikes occur constantly in a skid-steer whenever the wheels alternate between sticking and slipping during a turn. These spikes create massive current draws on your battery and stress your gearboxes. Your slew rate limits and watchdog settings inside the Motor Output Conditioner must be configured tightly to smooth out these sudden acceleration jumps without introducing control lag into your path tracking.
## 3. Effective Turning Geometry

A skid-steer robot rarely rotates according to its exact physical wheel spacing because turning requires tire scrub. The effective turning geometry depends on wheelbase, track width, tire compound, surface friction, robot weight distribution, and chassis compliance.

For this reason, the tank backend should use an empirical configuration value such as `tank_eff_wheel_base_4wd_m` rather than relying only on the physical wheel spacing. This value is tuned so commanded `angular_z_rps` produces the expected measured rotation rate.

---

# `vel_mecanum.py`

## Purpose

Closed-loop 4WD mecanum / holonomic velocity tracking.

This backend is for mecanum robots that can move in:

```text
linear_x   forward/backward
lateral_y  left/right strafe
angular_z  yaw rotation
```

## Output prefix

```text
mec_*
```

Example outputs:

```text
mec_motor_power_front_left
mec_motor_power_front_right
mec_motor_power_rear_left
mec_motor_power_rear_right
```

## Typical command inputs

```text
mec_cmd_vel_linear_x_mps
mec_cmd_vel_lateral_y_mps
mec_cmd_vel_angular_z_rps
```

## Typical feedback inputs

```text
mec_robot_linear_x_mps
mec_robot_lateral_y_mps
mec_robot_angular_z_rps
```

These feedback signals come from localization/source IO. The backend should not know whether they came from dead wheels, vision, IMU, drive encoders, or sensor fusion.

## Typical configuration

```text
mec_wheel_base_4wd_m
mec_wheel_track_4wd_m
mec_wheel_radius_4wd_m
mec_pid_gains_linear_x
mec_pid_gains_lateral_y
mec_pid_gains_angular_z
mec_velocity_limit_linear_x_mps
mec_velocity_limit_lateral_y_mps
mec_velocity_limit_angular_z_rps
vel_watchdog_timeout_ms
```

## How it works

The backend compares commanded body velocity against measured body velocity:

```text
linear_x_error  = cmd_vel_linear_x_mps  - robot_linear_x_mps
lateral_y_error = cmd_vel_lateral_y_mps - robot_lateral_y_mps
angular_z_error = cmd_vel_angular_z_rps - robot_angular_z_rps
```

The controller produces corrected body-space effort, then applies mecanum inverse kinematics to produce four raw motor power commands.

Conceptually:

```text
vx, vy, wz command
        ↓
body velocity error correction
        ↓
mecanum inverse kinematics
        ↓
mec_motor_power_*
```

## When to use

Use this backend for:

- mecanum robots
- holonomic path tracking
- autonomous strafing
- robots that need independent X, Y, and yaw control

## When not to use

Do not use this backend for differential or tank robots that cannot strafe.

## Notes

Open-loop timed mecanum movement is usually not recommended for accurate autonomous control because mecanum motion is highly sensitive to wheel slip and surface conditions. Closed-loop body velocity control is the preferred approach.

# Velocity Mecanum Design Considerations

## 1. Holonomic Motion Requires Better Feedback

Mecanum can command `linear_x`, `lateral_y`, and `angular_z`, but the actual robot motion is highly affected by roller slip, floor surface, wheel loading, and frame alignment.

## 2. Lateral Motion Is Usually Weaker

Strafing is typically less efficient and less repeatable than forward motion. `mec_pid_gains_lateral_y` and `mec_velocity_limit_lateral_y_mps` may need to be more conservative than the X-axis values.

## 3. Geometry Must Match Wheel Layout

`mec_wheel_base_4wd_m`, `mec_wheel_track_4wd_m`, and `mec_wheel_radius_4wd_m` define the inverse kinematics. Incorrect dimensions will show up as unwanted rotation, diagonal drift, or poor strafe behavior.

## 4. Normalize Combined Motion

When commanding X, Y, and rotation at the same time, one or more wheel commands may exceed the allowed range. The Motor Output Conditioner should proportionally normalize outputs so the direction of the requested motion is preserved.

## 5. Best Use Case

This backend is best for robots that require precise holonomic movement, lateral path correction, docking, alignment, or field-relative navigation.

---

# Coordinate Convention

This package assumes a standard robot body-frame convention:

```text
+X = forward
+Y = left / lateral strafe
+Z = up
angular_z = yaw rotation about the vertical axis
```

Typical velocity signals:

```text
robot_linear_x_mps
robot_lateral_y_mps
robot_angular_z_rps
```

2WD differential and 4WD tank robots use:

```text
linear_x
angular_z
```

Mecanum robots use:

```text
linear_x
lateral_y
angular_z
```

---

# Feedback Philosophy

Closed-loop velocity backends should use robot/body velocity feedback, not direct physical encoder bindings.

The backend should consume signals such as:

```text
robot_linear_x_mps
robot_lateral_y_mps
robot_angular_z_rps
```

Those signals are provided by localization/source IO.

The motion backend should not know whether velocity feedback came from:

- drive encoders
- deadwheel odometry
- IMU
- vision
- sensor fusion
- dead reckoning

That decision belongs to the localization system and localization arbiter.

---

# Output Naming Convention

Each backend uses a prefix to identify its raw output source:

```text
drv_*   timed drive
rot_*   timed rotate
diff_*  differential velocity backend
tnk_*   tank velocity backend
mec_*   mecanum velocity backend
mux_*   selected backend output from motion_mux.py
cond_*  conditioned output from MotorOutputConditioner
```

The typical output chain is:

```text
backend-specific motor power
        ↓
mux_motor_power_*
        ↓
cond_motor_power_*
        ↓
Level2 / HAL
```

---

# Motion Backend vs Motor Output Conditioner

## Motion Backend

Responsible for:

- motion math
- velocity control
- drivetrain kinematics
- open-loop timing
- body velocity correction
- producing raw normalized motor effort

Not responsible for:

- polarity
- e-stop
- deadband
- slew/ramp limiting
- hardware pins
- motor board protocol

## Motor Output Conditioner

Responsible for:

- e-stop / motion inhibit
- output normalization
- motor deadband
- slew/ramp rate limiting
- max power clamp
- motor polarity
- disabled motor channels

Not responsible for:

- path tracking
- robot kinematics
- PID velocity correction
- localization
- hardware pin mapping

---

# Recommended Processing Order

The full motion pipeline should generally follow this order:

```text
1. Path tracking or command generator produces desired robot velocity
2. Selected motion backend computes raw motor power
3. Motion mux selects one backend output
4. Motor output conditioner applies safety/conditioning
5. Level2 maps canonical motor names
6. HAL/IOMap sends commands to physical motor hardware
```

---

# Safety Behavior

If a backend command is stale, invalid, or unavailable, the backend should output zero motor power.

If the mux selection is invalid, the mux should output zero motor power.

If `e_stop` is active, the motor output conditioner should force all motor outputs to zero.

Software e-stop in the motion layer does not replace a physical hardware e-stop. A real robot should still have a hardware-level emergency stop or power cut mechanism.

---

# Practical Backend Selection

## Use `timed_drive.py` when:

- the robot is simple
- no useful velocity feedback exists
- movement precision is low priority
- testing basic drivetrain behavior

## Use `timed_rotate.py` when:

- simple programmed turns are sufficient
- heading feedback is unavailable
- testing polarity or turn direction

## Use `vel_diff_2wd.py` when:

- the robot is true 2WD differential drive
- motion is limited to forward/reverse plus rotation
- localization provides usable `linear_x` and `angular_z` velocity

## Use `vel_tank_4wd.py` when:

- the robot is 4WD skid-steer/tank drive
- left and right wheel groups are controlled together
- lateral strafing is not possible

## Use `vel_mecanum.py` when:

- the robot has mecanum wheels
- lateral strafing is required
- full holonomic velocity control is needed

---

# Implementation Notes

## Keep backend code small and focused

Each backend should:

1. Read its command inputs
2. Read its feedback inputs if closed-loop
3. Apply velocity limits
4. Compute raw motor power
5. Output backend-prefixed motor power signals

Each backend should avoid performing output conditioning directly.

## Use clear timeout behavior

`vel_watchdog_timeout_ms` should define how long the backend may continue using a velocity command before considering it stale.

When a command is stale, output zero motor power.

## Use backend-specific configuration prefixes

Using prefixes helps prevent configuration collisions:

```text
diff_*
tnk_*
mec_*
drv_*
rot_*
```

Shared output conditioning settings should remain outside backend files.

---

# Example High-Level Flow

## Mecanum path tracking

```text
PathTrackingMecanum
        ↓
mec_cmd_vel_linear_x_mps
mec_cmd_vel_lateral_y_mps
mec_cmd_vel_angular_z_rps
        ↓
vel_mecanum.py
        ↓
mec_motor_power_front_left
mec_motor_power_front_right
mec_motor_power_rear_left
mec_motor_power_rear_right
        ↓
motion_mux.py
        ↓
mux_motor_power_*
        ↓
MotorOutputConditioner
        ↓
cond_motor_power_*
        ↓
Level2 / HAL
```

## Non-mecanum path tracking

```text
PathTrackingNonMec
        ↓
nonmec_cmd_vel_linear_x_mps
nonmec_cmd_vel_angular_z_rps
        ↓
vel_diff_2wd.py or vel_tank_4wd.py
        ↓
diff_* or tnk_* motor power outputs
        ↓
motion_mux.py
        ↓
mux_motor_power_*
        ↓
MotorOutputConditioner
        ↓
cond_motor_power_*
        ↓
Level2 / HAL
```

## Open-loop primitive

```text
TimedDrive or TimedRotate
        ↓
drv_* or rot_* motor power outputs
        ↓
motion_mux.py
        ↓
mux_motor_power_*
        ↓
MotorOutputConditioner
        ↓
cond_motor_power_*
        ↓
Level2 / HAL
```

---

# Summary

The `motion_backend` package provides the robot-agnostic drivetrain execution layer.

It does not decide where the robot should go. That belongs to path tracking or command generation.

It does not decide where the robot is. That belongs to localization and the localization arbiter.

It does not talk to physical hardware. That belongs to Level2, HAL, IOMap, and MotorBoard.

Its job is to answer one question:

```text
Given this desired robot motion, what normalized motor power should be requested?
```

The mux selects the active answer. The motor output conditioner makes it safe. The HAL turns it into real actuator commands.
