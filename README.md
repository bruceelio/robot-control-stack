# Robot Project Architecture Overview

This project is structured around a **layered robotics architecture** that cleanly separates **decision-making**, **motion execution**, **hardware access**, and **testing**.

The goal is to make the system:

* Easier to reason about
* Safer to modify
* Testable without hardware
* Scalable to more complex behaviors

---

## High-Level Data Flow

```
HAL
 ↓
Motion Backends
 ↓
Primitives
 ↓
Navigation
 ↓
Behaviors
 ↓
Controller
 ↓
Main (Robot.py)
```

Each layer has a **single responsibility** and strict rules about what it may and may not do.

---

## Core Architectural Rules (Read This First)

**Golden rules:**

* **Behaviors do not touch hardware**
* **Navigation reasons about space, not motors**
* **Primitives execute, they do not decide**
* **The Controller orchestrates, it does not implement**
* **If it executes, it’s a primitive**
* **If it calculates, it’s control or planning**

If a file feels “confused,” it’s usually violating one of these rules.

---

## Layer-by-Layer Breakdown

### 1. HAL (Hardware Abstraction Layer)

**Purpose:**
Abstract real hardware into safe, consistent interfaces.

**What it does:**

* Reads sensors
* Writes motor outputs
* Defines pin mappings
* Shields the rest of the system from hardware quirks

**What it does NOT do:**

* Make decisions
* Contain logic or algorithms

**Location:**
`robot_project/hal/`

---

### 2. Motion Backends

**Purpose:**
Execute motion using a specific strategy (timed, encoder-based, simulated).

**What they do:**

* Enforce limits
* Execute motion commands
* Track progress

**What they do NOT do:**

* Decide where to go
* Interpret sensor meaning

**Location:**
`robot_project/motion_backends/`

---

### 3. Primitives

**Purpose:**
Provide **atomic robot capabilities**.

Primitives are the **only code allowed to directly cause physical action**.

**Examples:**

* Drive a distance
* Rotate an angle
* Grab or release an object
* Wait or halt safely

**What they do:**

* Execute a single action
* Report RUNNING / SUCCEEDED / FAILED

**What they do NOT do:**

* Choose goals
* Loop strategically
* Make decisions

**Rule of thumb:**

> If it decides, it’s not a primitive.

**Location:**
`robot_project/primitives/`

---

### 4. Navigation

**Purpose:**
Reason about **space, position, and goals**.

Navigation answers questions like:

* Where am I?
* Where are landmarks?
* What is the next spatial objective?

**What it does:**

* Pose estimation (localisation)
* Geometry and math
* Target selection
* Path and waypoint logic

**What it does NOT do:**

* Execute motion
* Control motors
* Read hardware directly

**Rule of thumb:**

> Navigation chooses *where*, not *how*.

**Location:**
`robot_project/navigation/`

---

### 5. Behaviors

**Purpose:**
Define **task-level robot actions**.

Behaviors coordinate primitives and navigation to achieve goals like:

* Escape wall
* Seek and collect an object
* Perform validation maneuvers

**What they do:**

* Sequence primitives
* Use navigation and perception
* Decide success/failure

**What they do NOT do:**

* Touch hardware
* Implement motor logic
* Switch other behaviors

**Rule of thumb:**

> One behavior, one job.

**Location:**
`robot_project/behaviors/`

---

### 6. Controller

**Purpose:**
The **central orchestrator** of the robot.

The controller:

* Chooses which behavior runs
* Manages robot state
* Handles mode transitions (INIT → SEARCH → COMPLETE)
* Enforces high-level rules

**What it does NOT do:**

* Implement behaviors
* Control motors
* Process raw sensors

**Rule of thumb:**

> The controller decides *what runs*, not *how it runs*.

**Location:**
`robot_project/robot_controller.py`

---

### 7. Main Entry Point (`Robot.py`)

**Purpose:**
System startup and shutdown only.

**What it does:**

* Initializes hardware
* Loads configuration
* Creates the Controller
* Starts the control loop

**What it does NOT do:**

* Make robot decisions
* Contain navigation or behavior logic

**Rule of thumb:**

> If logic lives in `main`, something is wrong.

---

## level2_canonical.py

**“The canonical robot capability API.”**

This file defines the **standard motion interface** used across the system.

It sits above HAL and below behaviors, providing:

* A stable, predictable motion API
* Consistency across simulation and real hardware
* A single place to reason about robot capabilities

---

## Testing Philosophy

Tests live in `robot_project/tests/` and follow these principles:

* Tests should fail **before the robot does**
* Logic should be testable without hardware
* Every serious bug earns a regression test

Testing can be enabled explicitly via configuration and never runs by accident.

---

## Why This Architecture Exists

This structure is designed to:

* Support rapid iteration without fear
* Allow aggressive refactoring
* Enable simulation-first development
* Scale from simple demos to complex autonomy

Most importantly:
**It keeps reasoning about the robot understandable.**

---

## Final Note to Future You

If you’re unsure where something belongs, ask:

1. Does it execute hardware? → **Primitive**
2. Does it reason about space? → **Navigation**
3. Does it sequence actions? → **Behavior**
4. Does it choose what runs? → **Controller**

Follow the rules, and the system stays sane.

