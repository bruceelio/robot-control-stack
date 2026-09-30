# README_WEBOTS_ROBOT_SIM.md

SR2026 Webots / `webots_robot` profile — empirical notes and open calibration tasks.

**Purpose:** keep simulator-specific behaviour, test results, calibration decisions, and unresolved assumptions together. This is **not** a list of settings to copy to Bobbot. Update the measurements after each controlled run.

**Evidence key:** **Observed** = seen in our Webots run logs or current profile; **working setting** = configured/assumed rather than independently measured; **test needed** = not yet established.

## 1. Camera, FOV and AprilTags

| Topic | Current information | Status / next test |
|---|---|---|
| Horizontal camera FOV | Approximately **45°**, the current value used by pose-servoing experiments. | **Working setting**; verify actual visible marker-bearing limits. |
| Servoing visibility margin | Current experimental gate: `FOV / 2 − 3°` = **±19.5°** for a 45° FOV. | **Working setting**, not a measured detection guarantee. |
| AprilTag pose/vision cadence | Approach logs typically show observations at approximately 0.088 s intervals in the runs examined; logged perception age is about 0.05 s. | **Observed in specific runs**, not a guaranteed frame rate. |
| Two faces on one cube | Cube ID 162 can appear as two AprilTag observations with different bearings and distances. | **Observed**. Final approach can use their bearing midpoint. |
| Single-face approach | Cube ID 146 was brought to approximately 310 mm with almost zero bearing before the final pickup. | **Observed** in the 30 September runs. |
| Vision during rotation | The maximum angular speed that preserves reliable recognition has **not** been measured. | **Test needed** (Section 6). |

## 2. Rotation behaviour

| Topic | Current information | Status |
|---|---|---|
| Small two-face turn | At power 0.20, a commanded clockwise ~8.6° turn of ~0.064 s produced ~9.8° observed change in face bearings. | **Observed** repeatedly at this test point; residual midpoint ~−1.2°. |
| Very short turn calibration | Earlier longer durations produced substantial overshoot; do not assume a linear duration-to-angle relationship across power settings. | **Observed**; needs a dedicated calibration curve. |
| Rate/visibility limit | The normal servoing speed cap and vision-retention limit are different quantities. | **Test needed**. |

## 3. Ultrasonic and pickup geometry

| Situation | Observations | Interpretation / status |
|---|---|---|
| Single-face cube 146, final push | Centre ultrasonic readings **34, 34, 34 mm** in one run and **35, 34, 35 mm** in another. | **Observed before grasp**. |
| Two-face cube 162, earlier trials | Some runs gave **no centre return** despite a visually reported pickup at the 35 mm push. One earlier miss gave distant readings near 1.286 m. | **Observed**; lack of return alone cannot prove pickup failure. |
| Two-face cube 162, latest bumper-instrumented run | Centre readings **35, 35, 36 mm** even though only the right bumper was pressed. | **Observed**; near reading does not prove the cube remained held after retreat. |
| Incidence angle / reflective geometry | Echo reliability may depend on object angle and what is in the sensor beam. | **Hypothesis to quantify**, not an established cube-angle cutoff in our tests. |
| Held-cube verification via lift | Find servo positions where a held cube gives a distinct repeatable ultrasonic reading versus the empty mechanism. | **Test needed:** `tools/challenges/lift_ultrasonic_sweep.py`. |

**Calibration procedure:** first grasp, lift and reverse clear of any cube remaining on the floor. Sweep mechanically safe lift positions while holding a cube; record five readings at each position. Repeat with an empty gripper and clear foreground. Do not configure a `VERIFY_GRIP` distance window until the two cases are distinguishable.

### Lift-sweep results — to fill in

| Cube state | Servo position | Sensor | Valid/5 | Median (mm) | Spread (mm) | Notes |
|---|---:|---|---:|---:|---:|---|
| Held | TBD | ultrasonic.front | TBD | TBD | TBD | |
| Empty | TBD | ultrasonic.front | TBD | TBD | TBD | |

## 4. Contact switches and gripper/lift

| Topic | Current information | Status |
|---|---|---|
| Front bumpers | Exposed as `bumper.front_left` and `bumper.front_right`; the IO adapter reads simulated Arduino pins 10 and 11. | **Configured / observed**. |
| Face-on pickup | Both bumpers asserted during the final push of cube 146 and remained asserted at the end. | **Observed**. |
| Angled pickup | On cube 162, only the right bumper asserted during one 345 mm commanded final approach. | **Observed**. |
| Interpretation | Bumpers indicate **contact**, not successful attachment. One-sided contact can accompany sideways pushing. | **Observed / conservative interpretation**. |
| Gripper actuation | Simulated grab uses the vacuum output on PowerBoard H0; a successful command is not a physical grip-verification signal. | **Configured / observed**. |
| Lift carry | Webots profile `LIFT_CARRY_POSITION = 0.0` to avoid blocking the simulator camera. | **Configured**. |
| Lift up | Current grasp skill performs `Grab -> LiftUp`; commanded top position is +1.0 through Level2. | **Configured / observed**. |
| Lift verify | Best lift position, reading range and whether it obscures the camera are not calibrated. | **Test needed**. |

## 5. Timing and Webots physics

- **Observed:** original timed Level2 drive uses one `io.sleep(duration)` and then stops the motors. A 160 mm reverse requested at ~0.456 s completed in ~0.50 s of logged simulation time in the observed run.
- **Observed:** splitting the 345 mm final drive (nominal **0.955 s**) into sleeps with bumper reads between them extended elapsed simulation time to approximately **1.39 s**, even with a requested 64 ms sleep step; sampling log intervals were ~88 ms.
- **Hypothesis / diagnosis to verify:** simulated Arduino reads and/or scheduler/physics stepping consume simulation time not deducted by the initial remaining-duration loop. Do not treat a commanded 345 mm drive as a verified physical distance during this discrepancy.
- **Observed:** Webots sometimes prints `The current physics step could not be computed correctly` during extended cube contact. This warning does not on its own establish the cause.
- **Design requirement:** timed motion must use an elapsed clock appropriate to the environment (simulation clock in Webots, monotonic clock on real robots), not merely subtract requested sleep intervals. Confirm the clock API before implementing this shared fix.
- **Open test:** measure elapsed **simulation time**, actual wheel/robot motion and wall-clock time separately in the same drive, with and without sensor polling.

## 6. Planned rotation-versus-vision benchmark

Goal: establish the fastest rotational motion that maintains reliable AprilTag tracking in Webots, **not** simply the fastest that completes a turn.

1. Use the same `InitEscape` translation and starting heading each run; keep markers and camera unchanged.
2. Rotate through the same known angular sweep at several controlled angular speeds/power levels. Do not change both power and geometry at once.
3. Log frame timestamps, requested/observed heading, marker ID and bearing, first/last appearance, number of missed observations and reacquisition delay.
4. Compare the **marker-bearing interval in which each ID is detected**, the proportion of expected observations actually received, and whether any tag is skipped entirely.
5. Repeat at least three times per setting. Choose a practical vision-preserving operating speed from measured results; do not substitute the present `SERVOING_ANGULAR_MAX_RAD_S` for an empirical limit.

**Benchmark results:** TBD.

## 7. Other items to keep in this file

- Camera pitch, occlusion by the lift/cube and AprilTag detectability at the camera edges.
- AprilTag pose reliability at short range versus static detection at long range.
- High-cube versus low-cube pickup geometry and the lift's effective height/clearance.
- Actual final-drive distance against timed-drive calibration when the cube resists motion.
- Grip verification using held-cube ultrasonic measurements versus post-retreat AprilTag detection.
- Vision updates during long blocking operations, and any differences between simulation and real-hardware timing.
- Simulator-specific sensor availability and the mapping to `config.has_io(...)` / `io` collections.

## 8. Code and configuration references

- `config/profiles/webots_robot.py` — active simulated robot capabilities and geometry.
- `config/schema.py` — resolved configuration / `lift_carry_position`.
- `hw_io/hw_sr2026sim.py` — bumper, ultrasonic, lift, gripper and simulator sleep adapters.
- `level2/level2_canonical.py` — canonical lift and timed motion operations.
- `behaviors/pickup_object.py` — docking and final-push observations.
- `tools/challenges/lift_ultrasonic_sweep.py` — held/empty-lift calibration.
- Run logs from 30 September 2026: first pickup (cube 146) and two-face pickup (cube 162).

**Rule for updating this README:** enter raw observed values and the exact test setup first; promote them to configuration constants only after repeatable calibration.
