# Mini-Project: Battery Voltage Measurement and Motor Speed Compensation

## Title
Measuring battery voltage under load and compensating motor output to maintain consistent robot performance.

## Research Question
How does battery voltage variation affect motor speed, and can motor command scaling be used to reduce speed variation as battery voltage changes?

## Background
A 12V NiMH battery does not remain at a constant voltage during robot operation. Its voltage changes with state of charge and drops temporarily under load, especially during motor acceleration or stall conditions. Since DC motor speed is affected by applied voltage, the same PWM command may produce different motor speeds at different battery voltages.

This project investigates whether measuring battery voltage and adjusting motor commands in software can produce more consistent motor behavior.

## Hypothesis
If battery voltage decreases, then motor speed at a fixed PWM command will also decrease. If the PWM command is adjusted based on measured battery voltage, then the motor speed variation will be reduced.

## Variables

### Independent Variable
Battery voltage measured at the Arduino Mega analog input using a voltage divider.

### Dependent Variable
Motor speed, measured using encoder counts over time or another repeatable speed measurement.

### Controlled Variables
- Same motor and gearbox
- Same motor controller
- Same wheel or flywheel load
- Same test surface or mechanical load
- Same measurement interval
- Same robot wiring configuration
- Same PWM frequency and control mode
- Same battery type

## Equipment
- 12V NiMH battery
- Arduino Mega
- Raspberry Pi 4B
- Motor controller, such as RoboClaw or Cytron MDD20A
- DC motor with encoder, if available
- Voltage divider connected to Arduino analog input
- Multimeter for calibration
- Robot power distribution system
- Data logging script on Pi or Arduino

## Voltage Measurement Circuit
The battery voltage is measured using a resistor divider connected to an Arduino analog input.

Example divider:

```text
Battery +12V ── 33kΩ ──┬── Arduino analog input
                       │
                      10kΩ
                       │
Battery GND ───────────┴── Arduino GND
```

A small capacitor, such as 0.1 µF, may be added from the analog input to ground to reduce noise.

## Calibration Procedure
1. Power the robot from the main battery.
2. Measure actual battery voltage using a multimeter.
3. Record the Arduino analog reading.
4. Repeat at several battery voltages if possible.
5. Calculate the conversion from analog reading to battery voltage.
6. Compare calculated voltage with multimeter voltage.
7. Adjust the voltage conversion factor if required.

## Methodology

### Part 1: Baseline Motor Speed Test
1. Fully charge the battery.
2. Place the robot or test motor in a safe, repeatable test setup.
3. Run the motor at a fixed PWM command, for example 40%, 60%, and 80%.
4. Record:
   - timestamp
   - measured battery voltage
   - PWM command
   - motor encoder speed or count rate
5. Repeat the test as the battery voltage decreases.
6. Plot motor speed versus battery voltage for each PWM command.

### Part 2: Voltage Compensation Test
1. Choose a nominal voltage, for example 12.0V.
2. Measure the live battery voltage.
3. Scale the motor command using:

```text
adjusted_pwm = requested_pwm × nominal_voltage / measured_voltage
```

4. Limit the adjusted PWM to the allowed range.
5. Repeat the same motor tests used in Part 1.
6. Record the same data fields.
7. Compare compensated and uncompensated results.

## Data Collection Table

| Trial | Time (s) | Battery Voltage (V) | Requested PWM | Adjusted PWM | Encoder Count Start | Encoder Count End | Interval (s) | Speed (counts/s) | Notes |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 |  |  |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |  |  |

## Data Analysis

### Baseline Analysis
- Plot battery voltage against motor speed.
- Calculate the percentage speed change between high and low battery voltage.
- Identify whether speed decreases as voltage decreases.

### Compensation Analysis
- Plot motor speed before and after compensation.
- Compare speed variation with and without compensation.
- Calculate improvement using:

```text
speed_variation_percent = 100 × (max_speed - min_speed) / average_speed
```

A successful compensation method should reduce this percentage.

## Expected Results
Without compensation, motor speed is expected to decrease as battery voltage drops. With compensation, motor speed should remain more consistent, although compensation may be limited when the battery voltage becomes too low or the motor is near maximum command.

## Risks and Limitations
- Motor load may vary during testing.
- Battery voltage may sag briefly during acceleration.
- PWM compensation cannot create more voltage than the battery can supply.
- At high PWM values, there may be little or no headroom for compensation.
- Encoder measurement noise may affect calculated speed.
- Motor temperature may change performance over repeated tests.

## Safety Notes
- Keep wheels or mechanisms clear of hands and loose wires.
- Secure the robot or motor during bench testing.
- Use appropriate fusing on the battery and branch circuits.
- Stop testing if wiring, connectors, or regulators become hot.
- Avoid intentionally stalling motors for long periods.

## Conclusion Template
The results showed that battery voltage [did/did not] affect motor speed. At fixed PWM, motor speed changed by approximately ___%. After applying voltage compensation, motor speed variation changed to approximately ___%. Therefore, the hypothesis was [supported/not supported]. Further improvements could include closed-loop speed control using encoder feedback in addition to voltage compensation.

## Future Work
- Compare voltage compensation with encoder PID speed control.
- Test under different mechanical loads.
- Measure voltage sag during acceleration.
- Add filtering or averaging to the voltage measurement.
- Investigate whether compensation improves autonomous driving consistency.
