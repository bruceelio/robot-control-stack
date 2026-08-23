# Drive Timing Model Selection

## Purpose

This document summarises the timing models that may be used to estimate Bob Bot's travel distance from commanded motor power and run duration.

The aim is to use the simplest model that predicts the robot accurately enough over the range in which it will actually operate.

---

## High-Power, Long-Distance Motion

For high-power, long-distance motion, a single linear timing model is probably sufficient:

```text
d = mt + b
```

where:

- `d` is distance travelled;
- `t` is commanded drive duration;
- `m` is the effective steady travel speed;
- `b` is the fitted intercept that absorbs startup and stopping effects.

This is the same general form as:

```text
y = mx + b
```

where:

- `m` is the slope;
- `b` is the y-intercept.

Because the run is long, the acceleration phase is a small fraction of the total motion. A detailed exponential model is unlikely to provide enough practical improvement to justify the added complexity.

### Example

Suppose acceleration and braking together introduce a roughly fixed distance effect of 100 mm.

On a 3,000 mm run, this is only about 3%. A fitted intercept can account for much of this effect:

```text
distance = high_speed × duration + intercept
```

This is well suited to long, open-field movement.

---

## Low-Power, Short-Distance Motion

Low-power motion is less certain because a short movement may finish before the robot reaches steady speed.

However, this does not automatically mean that an exponential model is required.

The first question is whether the measured low-power data is sufficiently linear over the actual operating range. If Bob Bot normally uses low power for moves between 100 mm and 600 mm, only the accuracy over that range matters.

### Example: Approximately Linear Low-Power Motion

```text
30% power:

0.25 s →  70 mm
0.50 s → 145 mm
0.75 s → 220 mm
1.00 s → 294 mm
```

This is essentially linear, even if the underlying motor physics is not perfectly linear. A straight-line fit would be the appropriate engineering choice.

### Example: Curved Low-Power Motion

```text
30% power:

0.25 s →  20 mm
0.50 s →  75 mm
0.75 s → 150 mm
1.00 s → 235 mm
```

This shows a pronounced curved startup region. A single straight line would not represent the shortest movements well.

---

## Candidate Nonlinear Models

If the low-power data is not sufficiently linear, the two main candidates are a quadratic model and an exponential motor-response model.

### 1. Quadratic Distance Model

```text
d(t) = at² + bt + c
```

If the robot starts from zero distance, it may be possible to simplify this to:

```text
d(t) = at² + bt
```

or, if the data supports it:

```text
d(t) = at²
```

The corresponding speed is:

```text
v(t) = 2at + b
```

A quadratic therefore assumes that speed changes linearly with time, which corresponds to approximately constant acceleration.

#### Advantages

- Simple to fit.
- Easy to calculate.
- Easy to invert when finding duration from a requested distance.
- Likely to be adequate over a limited short-motion range.
- Requires relatively little calibration data.

#### Disadvantages

- The predicted speed continues increasing indefinitely.
- It does not naturally approach a maximum speed.
- It should only be used within the tested time and distance range.
- Its coefficients are mainly empirical rather than a complete drivetrain model.

For Bob Bot, these limitations may not matter if low-power motion is only used over a narrow duration range.

---

### 2. Exponential Motor-Response Model

A simplified motor speed response is:

```text
v(t) = v∞(1 - e^(-t/τ))
```

where:

- `v∞` is the steady-state speed at the selected power;
- `τ` is the drivetrain time constant.

Integrating speed gives distance:

```text
d(t) = v∞[t - τ(1 - e^(-t/τ))]
```

A practical fitted version may also include an offset:

```text
d(t) = v∞[t - τ(1 - e^(-t/τ))] + c
```

This model represents a robot that:

- starts close to zero speed;
- accelerates most strongly at the beginning;
- experiences gradually decreasing acceleration;
- approaches a steady speed.

At longer durations, the exponential term becomes small and the equation becomes approximately linear:

```text
d(t) ≈ v∞t - v∞τ
```

#### Advantages

- Better physical representation of a motor and drivetrain.
- Naturally transitions from startup curvature to steady linear travel.
- Its parameters have useful physical meaning.
- May describe both short and longer travel at one power setting.

#### Disadvantages

- More difficult to fit reliably.
- Requires measurements across both short and longer durations.
- More difficult to invert from requested distance to run time.
- Static friction, deadband, wheel slip and braking may still make the real robot differ from the ideal model.

---

## Relationship Between the Models

For very short durations, the exponential model behaves approximately like a quadratic.

The early part of the exponential distance response can be approximated as:

```text
d(t) ≈ (v∞ / 2τ)t²
```

Therefore, the quadratic model is not an unrelated alternative. It can be viewed as an approximation of the early acceleration portion of the exponential response.

This means a quadratic may model Bob Bot's short low-power movements accurately enough without requiring the full exponential equation.

---

## Likely Practical Model

Bob Bot will probably use two, or at most three, calibration regions:

```text
High power, long motion:
    d = m_high t + b_high

Low power, ordinary short motion:
    d = m_low t + b_low

Possibly very short motion:
    lookup table or quadratic correction
```

The third region may not be necessary.

---

## Recommended Selection Process

1. Gather repeated distance measurements at each selected power and duration.
2. Plot distance against commanded duration.
3. Fit a straight line first.
4. Examine the residual errors over the actual operating range.
5. Keep the linear model if the errors are acceptable.
6. If the shortest low-power movements show consistent curvature, fit a quadratic.
7. Compare an exponential model only if the data spans both startup and steady-speed travel, or if the quadratic does not provide sufficient accuracy.
8. Use the simplest model that provides acceptable real-world performance.

## Expected Outcome

The most likely result is:

- **High-power, long-distance travel:** linear.
- **Low-power, ordinary short-distance travel:** linear if testing supports it.
- **Very short low-power travel:** quadratic correction or lookup table if required.
- **Exponential model:** only if it produces a clear practical improvement across a wider range.

# Addendum – Velocity Feedforward

One additional benefit of the drive timing calibration is that it also establishes the robot's effective velocity at each tested motor power.

For the linear timing model:

```text
distance = mt + b
```

the slope is:

```text
m = Δdistance / Δtime
```

with units of:

```text
mm/s
```

This means the fitted slope (`m`) is the robot's measured steady-state velocity at that power level.

For example:

```text
Power    Effective Velocity

30%      280 mm/s
50%      610 mm/s
80%    1,050 mm/s
100%   1,180 mm/s
```

*(Values shown are illustrative only.)*

Rather than simply providing an open-loop timing equation, the calibration therefore produces a measured relationship between motor power and robot velocity.

```text
Motor Power
      ↓
Measured Velocity
```

---

## Feedforward Control

These measured velocities can later be used as the feedforward component of a closed-loop velocity controller.

Instead of asking the feedback controller to generate the entire motor command, the calibration provides an initial estimate of the motor power required to achieve the requested velocity.

Conceptually:

```text
Desired Velocity
        ↓
Estimated Motor Power (Feedforward)
        ↓
Feedback Controller
        ↓
Final Motor Command
```

The feedback controller then compensates for real-world effects such as:

- Battery voltage changes
- Robot mass
- Floor surface
- Wheel wear
- Drivetrain friction
- External disturbances

This allows the feedback controller to make only small corrections rather than generating the complete command from scratch.

Typical benefits include:

- Faster response
- Reduced tracking error
- Less integral wind-up
- Smoother motion
- Easier controller tuning

---

## Relationship Between the Timing Model and Feedforward

The fitted timing equation:

```text
distance = mt + b
```

contains two different pieces of information.

The slope (`m`) represents the robot's effective steady-state velocity and is therefore useful for velocity feedforward.

The intercept (`b`) represents the accumulated startup and stopping effects measured during calibration and is useful for predicting travelled distance during timed open-loop motion.

These should therefore be treated independently:

```text
m  → Velocity feedforward

b  → Open-loop timing correction
```
> **⚠️ CRITICAL IMPLEMENTATION NOTE**
>
> The intercept (`b`) should **only** be used for purely open-loop,
> time-based movements.
>
> Once Bob Bot transitions to closed-loop motion control, startup and
> stopping behaviour should be handled by the trajectory generator,
> feedforward model and feedback controller working together.
>
> Applying the empirical intercept (`b`) to a closed-loop controller
> effectively compensates for the startup transient twice, which can
> produce overshoot, startup jerks or unstable behaviour.


---

## Calibration Outputs

A single drive timing calibration therefore produces two valuable engineering datasets.

### 1. Open-loop Timing

```text
distance = velocity × duration + intercept
```

Used whenever the robot performs timed open-loop movement.

### 2. Velocity Feedforward

```text
Desired velocity
        ↓
Estimated motor power
```

Used as the initial command for future closed-loop velocity control.

---

## Summary

The drive timing calibration serves two purposes:

1. It calibrates accurate timed open-loop movement.
2. It creates an empirical drivetrain model that can be used as the feedforward component of future closed-loop velocity controllers.

This allows the same calibration data to support both simple timed driving and more advanced servoing controllers without requiring additional testing.