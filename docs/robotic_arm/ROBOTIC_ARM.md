# Modular Vision-Guided Robotic Arm Platform

## Overview

This project is based on the Hiwonder ArmPi FPV AI Vision Robotic Arm platform and is intended to serve as both a learning platform and a long-term robotic manipulation subsystem for future mobile robotics projects.

The project began as a concept for a custom-built vision-guided robotic arm intended for Student Robotics, FTC-style, and educational robotics applications. After evaluating the mechanical, electrical, and software requirements, the decision was made to adopt the Hiwonder ArmPi FPV as the baseline platform.

The ArmPi FPV provides:

* Proven mechanical geometry
* Proven bus servo architecture
* Proven vision integration
* Proven inverse kinematics
* ROS support
* OpenCV support
* Raspberry Pi integration
* Expandable software architecture

The project will initially focus on understanding and evaluating the stock platform before progressing toward a custom manipulation subsystem optimized for operation from a mobile robot.

The primary target object is currently:

* Cardboard cube
* Approximately 150 mm × 150 mm × 150 mm
* Approximately 150 g mass

---

# Project Objectives

The primary objectives of the project are:

* Learn robotic manipulation fundamentals
* Learn intelligent bus servo systems
* Learn robotic vision systems
* Learn ROS-based robotic architectures
* Learn inverse kinematics and coordinate transforms
* Develop a reliable object acquisition system
* Develop a modular robot-arm subsystem
* Support future mobile robot integration
* Support future custom robotic arm development
* Minimize unnecessary redesign effort

The project intentionally prioritizes:

1. Reliability over complexity
2. Proven hardware over custom hardware
3. Incremental development over full redesign
4. Learning over optimization
5. Practical performance over theoretical capability

---

# Development Philosophy

The project follows a staged development model.

Rather than immediately redesigning hardware, each subsystem will first be understood and characterized.

The philosophy is:

```text
Learn
    ↓
Measure
    ↓
Understand
    ↓
Modify
```

This approach minimizes engineering risk while maximizing knowledge gained from the platform.

---

# Development Phases

## Phase 1 – Platform Familiarization

The first phase focuses on understanding the ArmPi FPV exactly as supplied.

Objectives include:

* Assembly
* Calibration
* Vision testing
* Servo testing
* Inverse kinematics testing
* AI demonstrations
* ROS demonstrations
* OpenCV demonstrations
* Motion control evaluation

The goal is to fully understand the platform before introducing modifications.

---

## Phase 2 – Platform Characterization

The second phase focuses on gathering engineering data.

Measurements include:

* Reach
* Payload capability
* Current consumption
* Memory usage
* CPU usage
* Camera performance
* Servo temperatures
* Motion repeatability

The target object (150 mm cube) will be introduced during this phase.

---

## Phase 3 – End Effector Development

The stock gripper is designed for educational manipulation tasks involving small objects.

The primary target object for this project is significantly larger.

A custom end effector is expected to be developed.

Objectives include:

* Large capture envelope
* Lightweight construction
* Reduced vision precision requirements
* Improved pickup reliability
* Reduced gripping force requirements

The preferred concept is a guide-and-capture scoop style gripper.

---

## Phase 4 – Mobile Robot Integration

The arm will be integrated with a mobile robot platform.

The mobile robot will provide:

* Navigation
* Localization
* Long-range positioning
* Target approach

The arm will provide:

* Final target acquisition
* Fine positioning
* Object pickup
* Object placement

Typical workflow:

1. Mobile robot detects target.
2. Mobile robot approaches target.
3. Robot stops within approximately 20 cm.
4. Arm receives pickup request.
5. Arm performs acquisition.
6. Arm confirms completion.
7. Mobile robot resumes operation.

---

## Phase 5 – Lightweight Deployment

Once the platform is fully understood, the software architecture will be simplified.

The objective is to support deployment on a Raspberry Pi 4B with only 2GB of RAM.

This phase focuses on:

* Reducing memory usage
* Reducing boot time
* Eliminating unnecessary services
* Simplifying software architecture
* Improving reliability

---

# Mechanical Architecture

## Baseline Platform

The baseline platform is the Hiwonder ArmPi FPV.

Advantages include:

* Proven geometry
* Proven mechanical design
* Proven payload capability
* Proven software support
* Proven servo architecture

The stock platform will be preserved wherever practical.

---

## Future Modifications

Potential future modifications include:

* Custom gripper
* Reach optimization
* Mobile robot mounting interface
* Alternate camera systems
* Alternate end effectors
* Lightweight arm structures
* Custom arm redesigns

All modifications will be justified through testing.

---

# Servo Architecture

## Intelligent Bus Servos

The platform uses Hiwonder intelligent serial bus servos.

Known servo inventory includes:

| Servo   |
| ------- |
| LX-225  |
| LX-15D  |
| HTS-16L |

Additional spare servos have been acquired for experimentation.

Current inventory includes:

* ArmPi FPV servo set
* LX-225 spare
* LX-15D spare
* HTS-16L spare

---

## Bus Servo Advantages

The intelligent bus servo architecture provides:

* Position feedback
* Voltage feedback
* Temperature feedback
* Torque protection
* Shared communication bus
* Shared power rail
* Reduced wiring complexity

Compared to traditional PWM systems this dramatically simplifies robotic arm construction.

---

## Design Philosophy

The project follows the same philosophy used in the ArmPi FPV:

* High torque near the base
* Lower mass near the end effector
* Minimize moving mass
* Minimize power consumption
* Preserve efficiency

This architecture reduces shoulder loading and improves dynamic performance.

---

# Vision System

## Initial Vision Platform

The initial vision system will use the stock ArmPi FPV camera.

The stock system is expected to support:

* Object tracking
* Color recognition
* Coordinate transforms
* Pickup alignment

The stock camera will be evaluated before considering upgrades.

---

## Potential Future Camera Options

Potential future upgrades include:

* Raspberry Pi Camera 3
* Raspberry Pi Global Shutter Camera
* Arducam Lens Systems
* Alternate machine vision cameras

These upgrades will only be pursued if testing demonstrates a clear need.

---

## Vision Tasks

Vision responsibilities include:

* AprilTag detection
* Object localization
* Orientation estimation
* Pickup alignment
* Placement alignment
* Target verification

Vision is intended to reduce precision requirements elsewhere in the system.

---

# Electrical Architecture

## Development Configuration

The stock ArmPi FPV power architecture will initially be used without modification.

This allows evaluation of:

* Current consumption
* Servo loading
* Thermal performance
* Power stability

before any custom power architecture is developed.

---

## Future Mobile Robot Configuration

Long-term deployment is expected to use:

### Logic Power

* Raspberry Pi
* Camera
* Communication interfaces

### Servo Power

* Intelligent bus servos
* Dedicated servo power rail
* Independent protection

The exact power architecture will be finalized after characterization testing.

---

# Communication Architecture

## Standalone Operation

In standalone mode the robotic arm operates independently.

Responsibilities include:

* Vision processing
* Object detection
* Motion planning
* Servo control

This mode is intended for:

* Learning
* Development
* Calibration
* Testing

---

## Integrated Operation

In integrated mode the arm functions as a subsystem.

Communication may occur via:

* USB
* Serial
* Network interfaces

The arm is responsible only for manipulation tasks while the mobile robot handles navigation.

---

# Software Architecture

## Phase 1 Software

Initial software development will use the complete factory software environment.

This includes:

* ROS
* Docker
* OpenCV
* Factory calibration tools
* Factory AI demonstrations
* Factory inverse kinematics

The Raspberry Pi 4B 8GB will be used during this phase.

Objectives:

* Learn ROS
* Learn Docker
* Learn ArmPi architecture
* Learn coordinate systems
* Learn inverse kinematics
* Learn vision workflows

---

## Phase 2 Software

After the platform is understood, software will be progressively simplified.

Target architecture:

* Raspberry Pi OS Lite
* Headless operation
* Python
* OpenCV
* Intelligent bus servo control
* Minimal background services

Objectives:

* Lower memory usage
* Lower CPU usage
* Faster startup
* Simpler deployment
* Improved reliability

---

## Target 2GB Deployment

The final deployment target is:

* Raspberry Pi 4B
* 2GB RAM
* Headless operation

Expected software components:

```text
Vision
    ↓
Task Planner
    ↓
State Machine
    ↓
Motion Controller
    ↓
Bus Servo Interface
```

The final architecture should operate comfortably within a 2GB memory budget.

---

# State Machine Strategy

Initial operation will emphasize reliability.

Rather than relying heavily on advanced planning, operation will be based on predefined states.

Examples:

* HOME
* STOW
* SEARCH
* APPROACH
* ALIGN
* CAPTURE
* VERIFY
* LIFT
* PLACE
* RELEASE
* RECOVER

This simplifies debugging and improves robustness.

---

# AI Exploration

The ArmPi FPV includes AI-focused examples and demonstrations.

These will be explored during early development.

Areas of interest include:

* Object recognition
* Vision-guided manipulation
* Coordinate transforms
* Automated pickup routines
* Future machine-learning integration

AI experimentation is considered a learning objective rather than a deployment requirement.

The final mobile robot implementation is expected to rely primarily on deterministic algorithms wherever practical.

---

# Current Hardware Inventory

## Arm Platform

* Hiwonder ArmPi FPV
* Raspberry Pi 4B (8GB)

## Spare Servos

* LX-225
* LX-15D
* HTS-16L

## Accessories

* LX-15D Servo Bracket

---

# Current Status

Current Phase:

Phase 1 – Platform Familiarization

Immediate Objectives:

* Assemble ArmPi FPV
* Run factory software
* Complete calibration
* Explore AI demonstrations
* Explore ROS demonstrations
* Explore OpenCV demonstrations
* Evaluate stock camera
* Evaluate stock gripper
* Characterize system performance

Future Objectives:

* Design custom scoop gripper
* Integrate with mobile robot
* Develop lightweight software stack
* Validate Raspberry Pi 4B 2GB deployment
* Develop autonomous cube acquisition workflows

---

# Long-Term Vision

The long-term objective is not simply to build a robotic arm.

The objective is to develop a reusable robotic manipulation platform that can:

* Operate independently
* Operate from a mobile robot
* Support future arm designs
* Support future robotic research
* Support future educational robotics projects

The ArmPi FPV serves as both a capable robotic platform and a foundation for future development.

The project intentionally emphasizes learning, experimentation, and incremental improvement while preserving proven engineering solutions whenever practical.
