CHASSIS_SR_2040.md

# 2040 Extrusion Chassis Design Specification
**Project:** Student Robotics / Personal Robotics Platform

---

## 1. Design Philosophy

This chassis is intended to become the standard mechanical platform for future robots.

### Goals

- Simple to build using readily available UK components.
- Modular and serviceable.
- Low centre of gravity.
- Direct-drive drivetrain.
- Reusable electronics enclosure.
- Compatible with Student Robotics and future personal robots.

Unlike a goBILDA chassis, the 2040 extrusion provides the structure while industrial motors provide the drivetrain.

---

## 2. Overall Architecture

```text
Top Layer
----------------------------------------
Removable Electronics Box

----------------------------------------
3 mm Dibond Structural Deck

========================================
2040 Extrusion Frame

Motor Layer
----------------------------------------
37D Motors
Battery Tray
Rear Omni Assembly

----------------------------------------
Ground
```

The Dibond forms the torsion plate while the electronics box remains removable.

---

## 3. Structural Frame

### Extrusion

Use 2040 aluminium extrusion with the **40 mm dimension vertical**.

Benefits:

- Higher bending stiffness
- Better impact resistance
- Improved torsional rigidity

### Corner Construction

Use M4 corner brackets fixed with:

- M4 × 10 mm button head screws
- M4 spring T-nuts

---

## 4. Structural Floor

Use **3 mm Dibond** mounted **on top of the 2040 frame**.

Benefits:

- Continuous torsion box
- Flat mounting surface
- No motor cut-outs
- Easy electronics installation

---

## 5. Direct Drive Philosophy

```
37D Motor
    ↓
6 mm D Shaft
    ↓
goBILDA 1309 Sonic Hub
    ↓
96 mm Wheel
```

Advantages:

- Fewer parts
- Lower weight
- No couplers
- No external drive bearings
- Easy maintenance

External bearing support should only be added if testing demonstrates it is required.

---

## 6. Drive Motors

Preferred motor:

- DFRobot FIT0186
- 12 V
- 251 RPM
- 6 mm D shaft

Motors mount inside the chassis using commercial steel 37D mounting brackets.

---

## 7. Rear Omni

Preferred long-term solution:

```
37D Bracket
    ↓
Dummy 37D Bearing Cartridge
    ↓
6 mm Shaft
    ↓
6 mm Round Bore Sonic Hub
    ↓
96 mm Omni Wheel
```

The dummy cartridge should use two bearings and match the mounting geometry of the drive motors.

---

## 8. Battery

Battery mounted beneath the Dibond near the rear omni.

Benefits:

- Low centre of gravity
- Good anti-tip balance
- Easy removal

---

## 9. Electronics Box

A removable electronics box bolts to the Dibond.

Development electronics:

- Raspberry Pi
- Arduino Mega
- RoboClaw

Competition electronics can later replace the complete box.

---

## 10. Ground Clearance

Design targets:

- Wheel diameter: 96–97 mm
- Ground clearance: approximately 30 mm
- Flat Dibond deck on top of chassis

---

## 11. Fastener Standard

### M3

Electronics only.

### M4

Default mechanical fastener.

Preferred:

- M4 × 10 mm button head

Longer screws only when required.

Use penny washers only for Dibond or plastics.

---

## 12. Build Sequence

1. Assemble 2040 frame.
2. Install corner brackets.
3. Install motor brackets.
4. Install motors.
5. Install wheels.
6. Install Dibond deck.
7. Install rear omni.
8. Install battery tray.
9. Install electronics box.

---

## 13. Future Improvements

- Dummy 37D rear cartridge
- PETG-CF printed accessories
- Modular battery tray
- Modular front mechanism interface

---

## 14. Summary

This platform emphasises:

- Reliability
- Simplicity
- Serviceability
- Low centre of gravity
- UK sourcing
- Direct drive
- Modular construction

It intentionally combines industrial 2040 extrusion with industrial 37D gearmotors and the goBILDA wheel ecosystem to produce a reusable robotics platform.