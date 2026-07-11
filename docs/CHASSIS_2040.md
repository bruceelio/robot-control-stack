# 2040 Extrusion Chassis Design Notes for Student Robotics / Autonomous Robots

## 1. Orientation: Orient for the Load

### The Golden Rule

Always mount the 2040 profile with the **40 mm face standing vertically** around the perimeter of the chassis.

```text
Good:

   40 mm
     ↑
┌───────┐
│       │
└───────┘
   20 mm
```

### Why?

Extrusions are weakest against vertical bending. Standing the 40 mm face vertically maximizes the structural moment of inertia and significantly reduces chassis sag under the weight of:

* Batteries
* Arms and lifts
* Motors
* Electronics
* Payloads

This orientation provides the greatest stiffness for the least weight.

---

## 2. Corner Joint Construction (Anti-Racking)

Standard right-angle corner brackets are usually insufficient for a 4–7 kg autonomous robot that will experience repeated impacts with walls and field elements.

### Recommended: Double Corner Brackets

Use large gusset brackets that engage both vertical slots.

Examples:

* 4-hole gussets
* 6-hole gussets

```text
2040 ========
      ||
      ||
      ||
      ||
    2040
```

Benefits:

* Better resistance to racking
* Higher torsional rigidity
* Improved impact resistance

---

### Recommended: Through-Bolted End Joints

For maximum strength:

1. Tap the center hole of one extrusion end (typically M5).
2. Drill a clearance hole through the side of the mating extrusion.
3. Insert a long M5 button-head screw through the clearance hole.
4. Thread directly into the tapped extrusion.

```text
Top View

===============
      |
      |  M5 Bolt
      V
===============
```

Benefits:

* Strongest practical extrusion joint
* Fewer external brackets
* Cleaner appearance
* Lower part count

This technique is especially attractive once a drill press is available.

---

## 3. Integrated Axle and Motor Mounting

This is the key difference between an extrusion-based robot and a goBILDA-style robot.

### goBILDA Philosophy

```text
Channel
 ↓
Bearing
 ↓
Shaft
 ↓
Wheel
```

The channel itself acts as a bearing support.

### 2040 Philosophy

```text
2040
 ↓
Bearing Block
 ↓
Shaft
 ↓
Wheel
```

The extrusion is only structure.

Bearing support is provided by separate components.

---

## 4. The Slotted Pillow Block Trick

One of the most elegant drivetrain solutions.

### Components

* 2040 extrusion
* KP08 pillow blocks
* 8 mm silver steel shaft
* Wheel
* Chain or belt drive

Mount the KP08 pillow block directly into the lower slot of the 2040 using M5 T-nuts.

```text
Side View

Wheel
  O
  |
[KP08]
  |
=================
      2040
=================
```

### Advantages

* No custom bearing plates
* No machining required
* Easy alignment
* Easy replacement

---

### Built-In Chain Tensioning

Because the pillow block can slide along the T-slot:

```text
Loosen bolts
Slide assembly
Retighten bolts
```

This allows chain or belt tension adjustment without dedicated tensioners.

```text
Motor ---> Chain ---> Axle

Move axle assembly
to adjust tension
```

This is one of the strongest arguments for using extrusion in a drivetrain.

---

## 5. Motor Tucking

Industrial gearmotors pair particularly well with 2040.

Typical motor:

* 37 mm spur gearbox
* 37 mm diameter motor body

Because the rail is 40 mm tall:

```text
End View

┌──────────────┐
│    Motor     │
└──────────────┘
     2040
```

The motor can remain inside the chassis envelope.

Benefits:

* Better protection from impacts
* Reduced snagging
* Improved packaging
* Cleaner layout

This is particularly valuable for robots operating autonomously and potentially contacting walls.

---

## 6. Structural Floor Plates (Torsion Box Construction)

Do not rely solely on the extrusion frame.

Add a structural skin.

### Recommended Materials

* 4 mm Dibond
* 6 mm Birch Plywood
* 3 mm Aluminium Sheet
* Thick Polycarbonate

### Installation

Mount the sheet to the perimeter frame using M5 screws and T-nuts every 100 mm.

```text
Top View

+-------------------+
|                   |
| Structural Plate  |
|                   |
+-------------------+

Supported by 2040 frame
```

---

### Why It Works

The sheet transforms the frame into a torsion box.

Benefits:

* Eliminates diagonal twisting
* Reduces racking
* Increases stiffness dramatically
* Often removes the need for diagonal bracing

This is one of the highest stiffness-per-pound improvements available.

---

## 7. Comparison to goBILDA

### goBILDA

Strengths:

* Integrated ecosystem
* Bearings mount directly to structure
* Fast assembly
* Excellent for rapid iteration

Weaknesses:

* Higher cost
* Proprietary hardware
* Less flexible motor selection

---

### 2040 + Industrial Hardware

Strengths:

* Low cost
* Commodity hardware
* Easy local sourcing
* Compatible with industrial motors
* Excellent chassis platform

Weaknesses:

* Requires motor mounts
* Requires bearing supports
* More fabrication effort

---

## 8. Recommended Student Robotics Chassis Architecture

```text
2040 perimeter frame

+ KP08 pillow blocks

+ 8 mm silver steel shafts

+ 37 mm industrial gearmotors

+ 05B chain or HTD belt drive

+ Dibond or plywood torsion plate

+ PETG-CF printed motor mounts

+ PETG-CF printed sensor mounts

+ Raspberry Pi electronics stack
```

This approach emphasizes:

* Reliability
* Serviceability
* Low cost
* Local sourcing
* Autonomous robotics development

rather than FTC-style ecosystem integration.
