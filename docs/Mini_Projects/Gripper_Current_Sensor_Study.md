# Mini-Project: Servo Gripper Current Measurement and Grip Stop Detection

## Title
Measuring servo current during gripper closure and using current rise to detect object contact before crushing.

## Research Question
Can servo current measurement be used to detect when a gripper has contacted an object, and can this measurement be used to stop the gripper before damaging a cardboard cube?

## Background
A hobby servo draws different amounts of current depending on the mechanical load placed on it. When a gripper closes freely, the servo current should remain relatively low. When the gripper contacts an object, the servo must produce more torque, causing current to increase. If the current rise can be detected reliably, it may be possible to stop the gripper automatically before excessive force is applied.

This project investigates whether a current sensor connected in series with one gripper servo can detect object contact during closing. The first test uses only one gripper servo so that the wiring, calibration, software filtering, and stop threshold can be evaluated before testing both sides of the gripper.

## Hypothesis
If the gripper servo contacts the cardboard cube while closing, then the measured servo current will increase compared with free movement. If the Arduino stops the servo when current rises above a calibrated threshold, then the gripper will stop before crushing the cardboard cube.

## Variables

### Independent Variable
Gripper condition during closing:
- no object present
- cardboard cube present
- different current stop thresholds

### Dependent Variable
Servo current measured by the current sensor and read by the Arduino analog input.

### Controlled Variables
- Same servo
- Same gripper mechanism
- Same servo supply voltage
- Same current sensor
- Same Arduino analog input
- Same servo closing speed
- Same starting angle
- Same closing direction
- Same cardboard cube size
- Same measurement interval
- Same test surface and object position

## Equipment
- Arduino Mega
- One goBILDA 2000 Series Dual Mode Servo, 25-2 torque version
- 6V servo power distribution board or equivalent servo power feed
- 6V UBEC or regulated servo power supply
- ACS712 20A current sensor module or similar analogue current sensor
- 15 cm × 15 cm × 15 cm cardboard cube
- Multimeter for checking supply voltage and wiring
- Jumper wires or servo extension leads
- Data logging script on Arduino or Raspberry Pi
- Computer for serial monitoring and data capture

## Current Measurement Circuit
The ACS712 current sensor is inserted in series with only the positive power wire feeding the test servo.

```text
Servo PDB +6V ───────▶ ACS712 IP+
ACS712 IP- ──────────▶ Servo +6V / red wire

Servo PDB GND ───────▶ Servo GND / black or brown wire
Arduino PWM ─────────▶ Servo signal / yellow, orange, or white wire

ACS712 VCC ──────────▶ Arduino 5V
ACS712 GND ──────────▶ Arduino GND
ACS712 OUT ──────────▶ Arduino analog input, for example A0
```

The Arduino ground, servo power ground, and current sensor ground must be common.

```text
Arduino GND = Servo PDB GND = UBEC GND = ACS712 GND
```

Only the positive servo power wire passes through the ACS712. The servo ground and servo signal wires do not pass through the current sensor.

## Calibration Procedure
1. Power the Arduino and current sensor.
2. Leave the servo disconnected from the ACS712 output, or ensure no current is flowing through the sensor.
3. Record the zero-current analogue reading from the ACS712.
4. Repeat several readings and calculate the average zero-current value.
5. Connect the servo through the ACS712.
6. Run the servo with no load and record the current sensor reading during free movement.
7. If possible, compare the measured current with a multimeter or bench supply current reading.
8. Adjust the current conversion factor in software if required.
9. Record the normal free-moving current range before testing with the cube.

## Methodology

### Part 1: Sensor Baseline Test
1. Connect one gripper servo through the ACS712 current sensor.
2. Upload test code that logs:
   - timestamp
   - servo angle
   - raw analogue reading
   - calculated current
   - stop state
3. With no object in the gripper, move the servo from the open position toward the closed position in small steps.
4. Record the current during free movement.
5. Repeat at least three times.
6. Identify the typical current range during unloaded gripper movement.

### Part 2: Object Contact Test Without Automatic Stop
1. Place the 15 cm cardboard cube in the gripper test position.
2. Close the gripper slowly using small servo angle steps.
3. Record the current as the gripper approaches and contacts the cube.
4. Stop the test manually before visible crushing occurs.
5. Repeat at least three times.
6. Compare the current profile with the no-object baseline.
7. Identify the current increase that occurs at first contact.

### Part 3: Automatic Stop Threshold Test
1. Choose an initial current threshold slightly above the no-object free-movement current.
2. Program the Arduino to stop the servo when current exceeds the threshold for several consecutive readings.
3. Place the cardboard cube in the test position.
4. Close the gripper slowly.
5. Record whether the servo stops:
   - before contact
   - at light contact
   - after visible compression
   - too late, causing crushing
6. Adjust the threshold and repeat until the gripper stops at light contact without crushing the cube.

### Part 4: Repeatability Test
1. Use the best threshold from Part 3.
2. Run at least five repeated grip tests on the cardboard cube.
3. Record the stop angle, peak current, and whether visible deformation occurs.
4. Evaluate whether the same threshold works consistently.

## Suggested Arduino Control Logic

```text
start at open angle
move servo toward closed angle in small steps
wait briefly after each step
read current sensor several times
calculate filtered current

if filtered current > threshold for N consecutive readings:
    stop servo movement
    optionally back off slightly
    hold current position
```

A threshold should not be triggered by a single current spike. A better method is to require the threshold to be exceeded for 3 to 5 consecutive readings.

## Data Collection Table

| Trial | Test Type | Time (s) | Servo Angle (deg) | Raw ADC Reading | Calculated Current (A) | Threshold (A) | Stop Triggered? | Cube Condition | Notes |
|---:|---|---:|---:|---:|---:|---:|---|---|---|
| 1 | No object baseline |  |  |  |  |  |  |  |  |
| 2 | No object baseline |  |  |  |  |  |  |  |  |
| 3 | Cube contact, no auto stop |  |  |  |  |  |  |  |  |
| 4 | Cube contact, auto stop |  |  |  |  |  |  |  |  |
| 5 | Cube contact, auto stop |  |  |  |  |  |  |  |  |

## Additional Summary Table

| Trial | Start Angle (deg) | Stop Angle (deg) | Max Current (A) | Average Free-Move Current (A) | Current Rise at Contact (A) | Visible Crushing? | Successful Stop? |
|---:|---:|---:|---:|---:|---:|---|---|
| 1 |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |

## Data Analysis

### Baseline Analysis
- Plot servo angle against measured current for the no-object test.
- Calculate the average free-movement current.
- Identify any current spikes caused by servo startup or mechanical friction.
- Determine the highest normal free-movement current.

### Contact Analysis
- Plot servo angle against measured current for the cube contact test.
- Identify the point where current first rises above the no-object baseline.
- Compare this point with visual contact between the gripper and cube.
- Estimate the difference between free-movement current and contact current.

### Threshold Analysis
- Compare different current thresholds.
- Identify the lowest threshold that avoids false stops during free movement.
- Identify the highest threshold that does not visibly crush the cardboard cube.
- Choose a working threshold between these two limits.

A useful calculation is:

```text
current_rise = contact_current - average_free_movement_current
```

Another useful measure is:

```text
margin_above_baseline = stop_threshold - maximum_free_movement_current
```

A successful threshold should be high enough to avoid false triggering but low enough to stop before visible cube deformation.

## Expected Results
During free movement, the servo current is expected to remain relatively low, with brief spikes during acceleration. When the gripper contacts the cardboard cube, the current is expected to rise as the servo load increases. A correctly selected current threshold should stop the servo shortly after contact and before the cardboard cube is visibly crushed.

## Risks and Limitations
- Servo current is an indirect measure of grip force.
- Friction in the gripper mechanism may cause current increases even without object contact.
- Servo startup current spikes may cause false triggering.
- The ACS712 output may be noisy and may require averaging or filtering.
- Cardboard stiffness may vary between cubes.
- A single-servo test does not fully represent the final two-servo gripper behaviour.
- If the current threshold is too high, the cube may be crushed before the servo stops.
- If the threshold is too low, the gripper may stop before making useful contact.

## Safety Notes
- Keep fingers clear of the gripper during testing.
- Stop testing immediately if the servo stalls, wiring becomes hot, or the cardboard cube begins to crush.
- Use appropriate fusing or current protection on servo power circuits.
- Do not intentionally stall the servo for long periods.
- Check that the ACS712 current rating is suitable for expected servo current spikes.
- Ensure all grounds are common before powering the system.
- Secure loose wires so they cannot enter the gripper mechanism.

## Conclusion Template
The results showed that servo current [did/did not] increase when the gripper contacted the cardboard cube. During free movement, the average current was approximately ___ A, and the maximum free-movement current was approximately ___ A. During cube contact, the current increased to approximately ___ A. A threshold of ___ A [did/did not] stop the servo before visible crushing occurred. Therefore, the hypothesis was [supported/not supported]. Further improvements could include testing both gripper servos, adding per-servo current sensing, using a spring-loaded fingertip sensor, or adding vacuum assistance to reduce the gripping force required.

## Future Work
- Repeat the test with both gripper servos installed.
- Compare one-servo and two-servo current profiles.
- Add filtering or averaging to reduce false triggers.
- Test different cardboard cube positions and orientations.
- Test different servo closing speeds.
- Add a small servo back-off after contact detection.
- Compare current sensing with Hall-effect spring deflection sensing.
- Investigate whether suction cups can reduce the grip force needed to lift the cube.
