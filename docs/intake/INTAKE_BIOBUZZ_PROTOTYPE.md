# FTC BIOBUZZ Intake Prototype Project

## Objective

Build a modular intake test platform to evaluate multiple intake architectures for FTC BIOBUZZ pollen balls (~2.5" / 64mm diameter).

The goal is to identify the most effective intake concept before committing to a final robot design.

Candidate architectures:

1. 48mm Wheel Intake
2. Rubber Band Drum Intake
3. Silicone Tubing Drum Intake
4. Vector Wheel Transfer Stage

---

# Current Inventory

## Existing Structure

Already available:

* goBILDA U-channel
* Brackets
* Standoffs
* Hardware
* Spacers
* Collars
* Bearings

No additional structural purchases required.

---

## Ordered Components

| Qty | Part                     |
| --- | ------------------------ |
| 1   | 240mm 8mm REX Shaft      |
| 4   | 48mm Gecko Wheels        |
| 2   | 48mm Boot Wheels         |
| 8   | 16mm Intake Rollers      |
| 4m  | Silicone Tubing (2 × 2m) |

---

# Additional Test Components

## Rubber Band Drum

Recommended:

* No.69 Rubber Bands
* Approx. 150mm lay-flat
* Approx. 300mm circumference
* Approx. 6mm width

Initial quantity:

* 12–16 bands

---

# Test Platform Design

## Frame

Simple rectangular frame using existing U-channel.

### Top View

```text
================================
|                              |
|                              |
|      intake shaft            |
|                              |
================================
```

Components:

* Two long side rails
* Two cross members
* Adjustable bearing supports

---

# Critical Feature

## Adjustable Shaft Height

Provide vertical adjustment slots.

Target test positions:

| Height |
| ------ |
| 25mm   |
| 30mm   |
| 35mm   |
| 40mm   |

This variable is expected to have a major effect on performance.

---

# Intake A: Wheel Intake

Configuration:

```text
Boot Gecko Gecko Gecko Gecko Boot
```

Purpose:

* Baseline performance
* Corner pickup evaluation
* Wall pickup evaluation
* Transfer evaluation

---

# Intake B: Rubber Band Drum

Configuration:

```text
Disc | Band Zone | Disc
```

or

```text
Boot | Band Zone | Boot
```

Band Zone:

* 12–16 No.69 bands
* 10–12mm spacing

---

# Rubber Band Drum End Disc

## Initial Prototype

Print two identical discs.

### Parameters

| Parameter      | Value                  |
| -------------- | ---------------------- |
| Outer Diameter | 60mm                   |
| Thickness      | 6mm                    |
| Bore           | 8mm REX or 8.1mm round |
| Slots          | 12                     |
| Slot Width     | 4mm                    |
| Slot Depth     | 5mm                    |
| Material       | PETG                   |

---

# Onshape Instructions for Drum Disc

## Step 1 – Create Disc

Open a new Part Studio.

Select the Front Plane.

Create a sketch:

* Draw a circle centered on the origin
* Diameter = 60mm

Finish sketch.

Extrude:

* Depth = 6mm
* Operation = New

---

## Step 2 – Create Shaft Bore

### Option A (Recommended)

Use goBILDA 8mm REX FeatureScript if available.

Create:

* 8mm REX bore

### Option B (Quick Prototype)

Create sketch on front face.

Draw:

* Circle diameter = 8.1mm

Extrude Remove:

* Through All

A round bore is acceptable for early testing if shaft retention is handled by collars.

---

## Step 3 – Create One Band Slot

Create sketch on front face.

Draw one rectangle centered on the outer edge.

Dimensions:

* Width = 4mm
* Depth = 5mm

The rectangle should overlap the outside circumference.

Example:

```text
      ______
     |      |
     |      |
-----|      |-----
```

Finish sketch.

Extrude Remove:

* Through All

Result:

```text
    \____/
```

A simple locating notch.

---

## Step 4 – Circular Pattern

Use Circular Pattern.

Pattern:

* Slot cut feature

Axis:

* Center axis of disc

Instances:

* 12

Equal spacing:

* 360°

Result:

```text
          _
      _       _
   _             _

  _       O       _

   _             _

      _       _
          _
```

---

## Step 5 – Add Edge Chamfer

Select:

* Outer edges
* Slot edges

Apply:

* 0.5mm chamfer

Purpose:

* Prevent cutting rubber bands
* Improve durability

---

## Step 6 – Export

Material:

* PETG preferred

Print settings:

* 0.2mm layer height
* 25–30% infill
* 4 perimeter walls

Print two copies.

---

# Drum Layout

Use straight loops.

Preferred:

```text
Disc ==================== Disc
```

Avoid X-wrap for initial testing.

---

# Shaft Layouts

## Wheel Intake

```text
Collar
Boot
Gecko
Gecko
Gecko
Gecko
Boot
Collar
```

---

## Drum Intake

```text
Collar
Disc
Band Zone
Disc
Collar
```

or

```text
Collar
Boot
Disc
Band Zone
Disc
Boot
Collar
```

---

# Motor Speed Testing

Initial RPM targets:

| RPM | Purpose      |
| --- | ------------ |
| 400 | Conservative |
| 600 | Baseline     |
| 800 | Aggressive   |

Avoid very high RPM (>1200 RPM) during early testing.

---

# Test Procedure

## Test 1 — Straight Pickup

Approach ball head-on.

Measure:

* Pickup success rate
* Bounce-out rate

---

## Test 2 — Angled Pickup

Approximate:

```text
30°
```

Measure:

* Pickup consistency
* Recovery from poor alignment

---

## Test 3 — Wall Pickup

Place ball directly against a wall.

Measure:

* Acquisition success
* Time to capture

---

## Test 4 — Corner Pickup

Worst-case approach angle.

Measure:

* Acquisition success
* Ball centering behavior

---

## Test 5 — Multi-Ball Pickup

Introduce two balls.

Observe:

* Jamming
* Bounce-outs
* Double acquisition

---

## Test 6 — Transfer Evaluation

Observe:

* Ball control after acquisition
* Self-centering behavior
* Exit consistency

---

# Data Collection

Record:

| Variable                |
| ----------------------- |
| Intake Type             |
| Shaft Height            |
| RPM                     |
| Straight Pickup Success |
| Wall Pickup Success     |
| Corner Pickup Success   |
| Multi-Ball Performance  |
| Bounce-Out Rate         |
| Subjective Notes        |

Use slow-motion video whenever possible.

---

# Future Experiment C

## Silicone Tubing Drum

Only perform after rubber-band drum testing.

Maintain:

* Same geometry
* Same shaft height
* Same spacing

Change only:

* Rubber bands → Silicone tubing

Purpose:

* Compare durability
* Compare ball interaction
* Compare consistency

---

# Future Experiment D

## Vector Wheel Transfer Stage

Use as secondary transfer stage only.

### Concept

```text
Front Intake
      ↓
Vector Wheels
      ↓
Transfer Path
```

Purpose:

* Center balls
* Narrow flow path
* Improve indexing

Not intended as primary acquisition mechanism.

---

# Development Sequence

```text
48mm Wheel Intake
        ↓
Rubber Band Drum
        ↓
Select Winner
        ↓
Silicone Tubing Version
        ↓
Vector Wheel Transfer Stage
```

---

# Success Criteria

The preferred intake should demonstrate:

* Reliable straight pickup
* Reliable wall pickup
* Reliable corner pickup
* Low bounce-out rate
* Good multi-ball handling
* Consistent transfer behavior
* Simplicity of implementation

The final competition intake will be selected based on measured performance rather than assumptions.
