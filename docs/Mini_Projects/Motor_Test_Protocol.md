# REVS-004
# Motor Qualification and Validation Manual
**Revision 1.0 (Draft)**

---

# 1. Purpose

The purpose of this document is **not** to determine the theoretically best motor.

Its purpose is to provide sufficient engineering confidence to answer one question:

> **Can these two selected motors be trusted for the intended robot application?**

If the answer is **Yes**, proceed with robot integration.

If the answer is **No**, stop before committing significant engineering effort and investigate before replacing the motors.

---

# 2. Test Philosophy

The protocol is intended to be practical rather than academic.

Measurements should be:

- Repeatable
- Relevant
- Good enough to support engineering decisions

They do **not** need laboratory precision.

Example:

A radial load of approximately **1 kg ±10%** is perfectly acceptable if it is sufficient to determine whether unacceptable behaviour occurs.

The guiding principle throughout this manual is:

> **Will this measurement change an engineering decision?**

If not, it should not be part of the qualification protocol.

---

# 3. Equipment Required

Minimum equipment:

- Motor qualification bench
- Arduino Mega
- Dual motor driver
- Encoder inputs
- Current measurement
- Temperature measurement
- Variable PWM control
- Logic analyser (recommended)
- Radial loading fixture (basic)
- 100 mm backlash pointer
- Test wheels and hubs

---

# PHASE 1 – MOTOR QUALIFICATION

Every motor pair undergoes Phase 1.

Purpose:

> **Can I trust these motors?**

---

# Test 1 – Motor Pair Matching

## Purpose

Determine whether the selected pair behaves similarly enough for the intended robot.

## Procedure

- Run both motors simultaneously.
- Test low, medium and high PWM.
- Test forward and reverse.
- Record encoder speed.
- Record current.
- Record temperature.

## Additional Check

### Multi-position Breakaway

Repeat the breakaway test at:

- 0°
- 45°
- 90°
- 135°
- 180°
- 225°
- 270°
- 315°

## Pass

- Motors remain well matched.
- No significant position-dependent behaviour.

---

# Test 2 – Health Check

## Inspect

- Shaft play
- Shaft runout
- Gearbox noise
- Wiring
- Connectors
- Vibration

## Measure

- No-load current
- Temperature
- Encoder operation

## Baseline Backlash

Attach a 100 mm pointer to the output hub.

Measure total backlash before endurance testing.

This value becomes the baseline.

---

# Test 3 – Breakaway & Low-Speed

## Procedure

Increase PWM slowly from zero.

Repeat several times in both directions.

Record:

- Breakaway PWM
- Sustained-running PWM

## Pass

- Repeatable starts.
- Stable low-speed operation.

---

# Test 4 – Radial Load Screening

## Purpose

Screen for obvious problems caused by direct wheel mounting.

Approximate loading is sufficient.

Suggested stages:

- Unloaded
- Approximately 1 kg
- Approximately 2 kg
- Approximately 3 kg (if appropriate)

Observe:

- Current
- Speed
- Temperature
- Noise
- Binding

Repeat the multi-position breakaway test while loaded.

Remove the load and repeat the unloaded baseline.

---

# Test 5 – Endurance

Run a representative drive cycle for approximately 20–30 minutes.

Example:

- Accelerate
- Drive
- Stop
- Reverse
- Drive
- Stop

Repeat.

After completion repeat:

- Pair Matching
- Health Check
- Breakaway
- Backlash
- Encoder verification

Compare all results against the original baseline.

---

# Qualification Decision

| Result | Decision                           |
|---------|------------------------------------|
| Pass | Proceed with robot integration     |
| Minor understood issues | Proceed with documented mitigation |
| Significant concern | Proceed to Phase 3 Investigation   |

---

# PHASE 2 - REQUALIFICATION AFTER ROBOT SERVICE

Every motor pair that successfully completes **Phase 1 – Motor Qualification** shall undergo periodic requalification during its service life.

Purpose:

> **Can I continue to trust these motors?**

Unlike Phase 3, requalification is a routine maintenance activity and should form part of the normal robot development and competition cycle.

---

## When to Requalify

Requalification should normally be performed:

- After initial robot commissioning.
- After approximately **10–20 hours** of robot operation (initial recommendation).
- Following any significant collision or drivetrain impact.
- After any major drivetrain repair or modification.
- Before major competitions.
- Whenever reduced drive performance is suspected.

### Engineering Note

The recommended **10–20 hour** interval is intentionally conservative and should be refined as experience is gained with a particular motor platform.

The objective is to identify trends in motor condition rather than wait for failure.

---

## Requalification Procedure

Repeat the complete **Phase 1 – Motor Qualification Protocol**.

No additional testing is normally required at this stage.

Compare every measurement against the original qualification baseline.

Particular attention should be paid to:

- Motor pair matching
- Breakaway PWM
- Backlash
- No-load current
- Current under representative load
- Temperature
- Encoder stability
- Gearbox noise
- Shaft play
- Radial load behaviour

The comparison between the original qualification and the requalification is more important than the absolute measurements.

---

## Requalification Outcomes

### Outcome A – Continue in Service

**Conditions**

- Results remain substantially unchanged.
- No meaningful deterioration is observed.

**Action**

Continue normal robot operation.

---

### Outcome B – Continue with Increased Monitoring

**Conditions**

- Minor deterioration is observed.
- Performance remains acceptable for the intended robot.

**Action**

Continue operation.

Reduce the interval before the next requalification until confidence is restored.

---

### Outcome C – Proceed to Phase 3 Investigation

**Conditions**

One or more qualification measurements show meaningful deterioration and the cause is not immediately obvious.

**Action**

Carry out the appropriate **Phase 3 Diagnostic Investigation(s)** before replacing the motor.

---

### Outcome D – Replace Motor

**Conditions**

The cause of failure is obvious and indicates that the motor is no longer suitable for continued service.

Examples include:

- Severe gearbox damage.
- Excessive shaft play.
- Failed encoder.
- Permanent overheating.
- Mechanical failure.

**Action**

Replace the motor.

Further investigation is normally unnecessary unless similar failures are occurring across multiple motors.

---

## Engineering Philosophy

Requalification exists to answer one simple question:

> **Can I continue to trust these motors?**

Only if the answer is **No** should the engineer proceed to **Phase 3 – Diagnostic Investigation**.

Phase 3 is therefore an **exception process**, not a routine part of motor qualification.

---

## Requalification Workflow

```text
Phase 1 Qualification
        │
        ▼
Robot Build
        │
        ▼
Robot Service
(approximately 10–20 hours)
        │
        ▼
Requalification
        │
        ├── Pass
        │       │
        │       ▼
        │   Continue in Service
        │
        └── Concern Identified
                │
                ▼
      Phase 3 Investigation
                │
                ▼
      Apply Remediation
                │
                ▼
      Repeat Phase 1 Qualification
                │
                ▼
      Continue or Replace
```


# PHASE 3 – ROOT CAUSE INVESTIGATION

Only perform Phase 3 if Phase 1 identifies a concern.

Purpose:

> **Why has confidence been lost?**

Choose only the investigations relevant to the observed symptom.

---

# Investigation A – Backlash Quantification

Purpose:

Determine whether backlash has increased.

Method:

- 100 mm pointer
- Compare against baseline
- Compare before and after endurance

---

# Investigation B – Detailed Radial Loading

Purpose:

Determine whether radial loading causes gearbox or shaft problems.

Possible additions:

- Increased loading stages
- Longer duration
- Current monitoring
- Temperature monitoring

---

# Investigation C – Encoder Investigation

Purpose:

Investigate unstable encoder behaviour.

Possible tests:

- Logic analyser
- 50 ms velocity logging
- 100 ms logging
- 250 ms logging
- Quadrature transition checking
- Velocity ripple

---

# Investigation D – Current Investigation

Purpose:

Determine why current has increased.

Investigate:

- Wheel drag
- Shaft loading
- Gearbox friction
- Driver behaviour

---

# Investigation E – Thermal Investigation

Purpose:

Determine why temperature has increased.

Investigate:

- Duty cycle
- Loading
- Gearbox
- Bearings
- Controller settings

---

# Investigation F – Gearbox Inspection

Purpose:

Mechanical inspection.

Inspect:

- Gear wear
- Bearings
- Lubrication
- Shaft
- Encoder

---

# Choosing the Correct Investigation

| Qualification Finding | Investigation |
|-----------------------|---------------|
| Increased current | Current + Radial Load |
| Increased backlash | Backlash Quantification |
| Heating | Thermal Investigation |
| Speed mismatch | Pair Matching + Current |
| Encoder instability | Encoder Investigation |
| Gearbox noise | Gearbox Inspection |
| Binding | Radial Load Investigation |

---

# Decision Tree

```text
Phase 1 Qualification

        │
        ▼

Pass?

 ├── Yes → Proceed with Robot

 └── No

        │
        ▼

Select appropriate Phase 3 Investigation

        │
        ▼

Identify root cause

        │
        ▼

Apply remediation

        │
        ▼

Repeat Phase 1 Qualification

        │
        ▼

Pass?

 ├── Yes → Continue

 └── No → Replace motor or redesign
```

---

# Appendix A – Qualification Cover Sheet

- Motor Model
- Motor ID
- Date
- Operator
- Controller
- Firmware
- Battery
- Wheel
- Hub
- Ambient Temperature
- Comments

---

# Appendix B – Phase 1 Test Sheets

One record sheet for each of the five qualification tests including:

- Objective
- Equipment
- Measurements
- Pass / Fail
- Observations
- Notes

---

# Appendix C – Final Qualification Report

- Test 1
- Test 2
- Test 3
- Test 4
- Test 5

Overall Decision:

- Proceed
- Proceed with Mitigation
- Phase 2 Investigation
- Reject

---

# Appendix D – Motor Service History

| Date | Robot Hours | Qualification Result | Notes |
|------|------------:|----------------------|-------|