# Runtime Flow

This document describes what happens when the robot starts and how control flows during execution.

---

## Startup Sequence

1. Configuration is resolved once at startup
2. Hardware abstraction is initialised
3. Motion backend is selected
4. Controller is created
5. Main control loop begins

---

## High-Level Runtime Flow

Configuration (resolved once)
↓
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
Main Loop


All runtime logic ultimately flows through a **single immutable Config object**.

---

## Control Ownership

- Only primitives may execute motion
- Only navigation may reason about geometry
- Only behaviors may sequence actions
- Only the controller may decide what runs next

This strict ownership model keeps behavior predictable and debuggable.