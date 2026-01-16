# Robot Control Stack — Architecture

This project implements a **layered robotics control architecture** designed for:

- Simulation-first development
- Multiple robots with different hardware
- Multiple competitions and arenas
- Safe iteration without breaking working behavior

The system cleanly separates:

- Configuration
- Decision-making
- Motion execution
- Hardware access
- Testing and diagnostics

The goal is to keep the robot **understandable, debuggable, and evolvable**.

---

## Architectural Principles

**Golden rules:**

- Configuration is resolved once and never mutated
- Behaviors do not touch hardware
- Navigation reasons about space, not motors
- Primitives execute, they do not decide
- The Controller orchestrates, it does not implement
- If it executes, it’s a primitive
- If it calculates, it’s planning or control

If a file feels “confused”, it is almost always violating one of these rules.

---

## Layered Architecture

HAL
↓
Motion Backends
↓
Primitives
↓
Navigation & Perception
↓
Behaviors
↓
Controller
↓
Main


Each layer has **one responsibility** and strict rules about what it may and may not do.

---

## Layer Responsibilities

### HAL (Hardware Abstraction Layer)

**Purpose**  
Abstract real hardware into safe, consistent interfaces.

**Responsibilities**
- Read sensors
- Write motor outputs
- Define pin mappings
- Hide hardware quirks

**Must NOT**
- Make decisions
- Contain algorithms

**Location**

hal/


---

### Motion Backends

**Purpose**  
Provide concrete strategies for executing motion (timed, encoder-based, simulated).

**Responsibilities**
- Enforce motion limits
- Execute drive / rotate commands
- Handle timing or encoder feedback

**Must NOT**
- Decide where to go
- Interpret perception data

**Location**

motion_backends/


---

### Primitives

**Purpose**  
Provide **atomic robot capabilities**.

Primitives are the **only code allowed to cause physical action**.

**Examples**
- Drive a distance
- Rotate an angle
- Lift up / lift down
- Grab or release
- Wait safely

**Responsibilities**
- Execute one action
- Report RUNNING / SUCCEEDED / FAILED

**Must NOT**
- Choose goals
- Implement strategy
- Loop or plan

**Rule of thumb**
> If it decides, it’s not a primitive.

**Location**

primitives/


---

### Navigation & Perception

**Purpose**  
Reason about **space, position, and observed targets**.

**Responsibilities**
- Pose estimation
- Geometry and transforms
- Target filtering and ranking
- Distance and bearing calculations

**Must NOT**
- Execute motion
- Control motors directly

**Rule of thumb**
> Navigation chooses *where*, not *how*.

**Location**

navigation/
perception.py


---

### Behaviors

**Purpose**  
Define **task-level robot actions**.

**Examples**
- Escape wall
- Seek and collect
- Post-pickup realign
- Recover localisation

**Responsibilities**
- Sequence primitives
- Use navigation and perception
- Decide success or failure

**Must NOT**
- Touch hardware
- Implement motor logic
- Switch other behaviors

**Rule of thumb**
> One behavior, one job.

**Location**

behaviors/


---

### Controller

**Purpose**  
The **central orchestrator** of the robot.

**Responsibilities**
- Select which behavior runs
- Manage robot state
- Handle mode transitions
- Enforce global rules

**Must NOT**
- Implement behaviors
- Control motors
- Process raw sensor data

**Rule of thumb**
> The controller decides *what runs*, not *how it runs*.

**Location**

robot_controller.py


---

### Main Entry Point (`robot.py`)

**Purpose**  
Startup and shutdown only.

**Responsibilities**
- Initialise hardware
- Resolve configuration
- Create the controller
- Start the control loop

**Must NOT**
- Contain logic
- Implement navigation or behavior code

**Rule of thumb**
> If logic lives in `robot.py`, something is wrong.