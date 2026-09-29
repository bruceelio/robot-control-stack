# SR2026 Stage 1 Autonomous Program

This document describes the current autonomous state machine implemented by:

```text
autonomous/auto_sr2026_stage1.py
```

Its purpose is to show, step by step:

- which state is active;
- which `*.py` file/class is called;
- what information is passed into that step;
- what happens when the step succeeds;
- what happens when the step fails;
- which strategy data is carried forward to later steps.

The autonomous program itself does not own the shared robot hardware/runtime. `Controller` continues to own perception, localisation, motion backends, Level 2 I/O, camera processing, and the canonical `io.*` interface.

---

## 1. Overall autonomous loop

Normal autonomous operation is:

```text
INIT_ESCAPE
    |
    v
PREPARE_SEARCH
    |
    v
GLOBAL_OBJECT_SEARCH
    |
    v
SELECT_TARGET
    |
    v
SEEK_AND_COLLECT / APPROACH_OBJECT
    |
    v
PICKUP_OBJECT
    |
    v
POST_PICKUP_REALIGN
    |
    v
RECOVER_LOCALISATION
    |
    v
RETURN_TO_BASE
    |
    v
DROPOFF_OBJECT
    |
    v
POST_DROPOFF_REALIGN
    |
    +----------------------+
                           |
                           v
                    PREPARE_SEARCH
```

The cycle then repeats for the next object.

There are recovery loops around several stages. These are described below.

---

## 2. Entry point

### File

```text
autonomous/auto_sr2026_stage1.py
```

### Function

```python
run(controller)
```

The runner creates one `AutoSR2026Stage1` object and then continuously executes:

```python
while True:
    controller.tick()
    autonomous.update(controller)
```

`controller.tick()` updates the shared robot runtime. `autonomous.update(controller)` advances the autonomous state machine.

---

## 3. Initial state selection

The initial state depends on `STARTUP_SCRIPT`.

If:

```python
STARTUP_SCRIPT == StartupScript.NONE
```

the first state is:

```text
INIT_ESCAPE
```

If a startup script is configured, the first state is:

```text
SCRIPTED_START
```

---

# Step-by-step state machine

## 4. SCRIPTED_START

**State:** `RobotState.SCRIPTED_START`

**Primary file called:** `behaviors/scripted_start.py`

**Class:** `ScriptedStart`

Start:

```python
self.behavior = ScriptedStart()
self.behavior.start(config=CONFIG)
```

Update inputs:

```python
motion_backend=controller.motion_backend
lvl2=controller.lvl2
```

### Internal sequence

1. Read the configured startup script:
   - Config point: `STARTUP_SCRIPT`
   - File: `config/strategy.py`

2. Resolve the selected scripted routine:
   - Registry: `scripted/registry.py`

3. Run the selected `scripted/*.py` routine through `ScriptedStart`.

**On success**

```text
SCRIPTED_START -> SEEK_AND_COLLECT (behaviors/approach_object.py)
```

**On failure**

Current code uses the same route:

```text
SCRIPTED_START -> SEEK_AND_COLLECT (behaviors/approach_object.py)
```

There is currently no different recovery path for a failed scripted start.

> Current caveat: the normal target-selection architecture expects `SelectTarget` to choose the target and servo methodology before `ApproachObject` runs. Directly entering `SEEK_AND_COLLECT` from `SCRIPTED_START` bypasses `PREPARE_SEARCH`, `GLOBAL_OBJECT_SEARCH`, and `SELECT_TARGET`.

---

## 5. INIT_ESCAPE

**State:** `RobotState.INIT_ESCAPE`

**Primary file called:** `behaviors/init_escape.py`

**Class:** `InitEscape`

Start inputs:

```python
config=CONFIG
motion_backend=controller.motion_backend
```

Update inputs:

```python
lvl2=controller.lvl2
localisation=controller.localisation
motion_backend=controller.motion_backend
```

Purpose: initial robot preparation and escape from the starting position.

### Internal sequence

1. Initialise lift:
   - Bob_Bot: `LiftDown` -> `primitives/manipulation/liftdown.py`
   - Other robots: `LiftUp` -> `primitives/manipulation/liftup.py`

2. Drive the configured distance:
   - Config point: `INIT_ESCAPE_DRIVE_MM`
   - Called through: `primitives/composites/drive_then_rotate.py`
   - Drive primitive: `primitives/motion/drive.py`

3. Rotate the configured angle:
   - Config point: `INIT_ESCAPE_ROTATE_DEG`
   - Called through: `primitives/composites/drive_then_rotate.py`
   - Rotate primitive: `primitives/motion/rotate.py`

**On success**

```text
INIT_ESCAPE -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

**On failure**

```text
INIT_ESCAPE -> no state transition (autonomous/auto_sr2026_stage1.py)
```

There is currently no explicit `FAILED` branch in `auto_sr2026_stage1.py`.

---

## 6. PREPARE_SEARCH

**State:** `RobotState.PREPARE_SEARCH`

**Primary file called:** `skills/manipulation/prepare_search.py`

**Class:** `PrepareSearch`

Start inputs:

```python
lvl2=controller.lvl2
```

Update inputs:

```python
self.behavior.update()
```

Purpose: prepare the manipulator for searching for the next object.

### Internal sequence

1. Open the gripper / release vacuum:
   - Primitive: `Release`
   - File: `primitives/manipulation/release.py`

2. Lower the lift:
   - Primitive: `LiftDown`
   - File: `primitives/manipulation/liftdown.py`

3. Preparation complete.

**On success**

```text
PREPARE_SEARCH -> GLOBAL_OBJECT_SEARCH (behaviors/global_object_search.py)
```

**On failure**

```text
PREPARE_SEARCH -> no state transition (autonomous/auto_sr2026_stage1.py)
```

Failure routing is currently marked as TBD.

---

## 7. GLOBAL_OBJECT_SEARCH

**State:** `RobotState.GLOBAL_OBJECT_SEARCH`

**Primary file called:** `behaviors/global_object_search.py`

**Class:** `GlobalObjectSearch`

Before the first successful delivery:

```python
required_kind = None
```

Either object kind is sufficient to continue.

After the first successful delivery:

```python
required_kind = self.delivered_kind
```

Already delivered marker IDs are excluded:

```python
exclude_ids=self.delivered_ids
```

Start inputs:

```python
config=CONFIG
required_kind=required_kind
exclude_ids=self.delivered_ids
motion_backend=controller.motion_backend
localisation=controller.localisation
```

Update inputs:

```python
perception=controller.perception
motion_backend=controller.motion_backend
localisation=controller.localisation
```

### Internal sequence

1. Check the current perception data for an eligible object:
   - Required kind is applied if one has already been established.
   - IDs in `delivered_ids` are excluded.

2. If no eligible object is visible, rotate by the configured search step:
   - Rotate primitive: `primitives/motion/rotate.py`

3. Settle and check perception again.

4. Repeat the scan until:
   - an eligible object is visible; or
   - the configured search sweep / timeout is exhausted.

**On success**

```text
GLOBAL_OBJECT_SEARCH -> SELECT_TARGET (skills/perception/select_target.py)
```

**On failure**

```text
GLOBAL_OBJECT_SEARCH -> RECOVER_LOCALISATION (behaviors/recover_localisation.py)
```

Before recovery:

```python
self.recover_localisation_success_state = RobotState.PREPARE_SEARCH
```

So successful localisation recovery returns to:

```text
RECOVER_LOCALISATION -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

---

## 8. SELECT_TARGET

**State:** `RobotState.SELECT_TARGET`

**Primary file called:** `skills/perception/select_target.py`

**Class:** `SelectTarget`

Purpose:

```text
choose WHICH object to approach
choose HOW that object should be approached
```

The selected servo methodology is carried separately from the raw perception target.

Current servo methodologies include:

```text
target_bearing
pose_bearing
```

Before the first delivery:

```python
required_kind = None
preferred_kind = CONFIG.default_target_kind
preferred_elevation = CONFIG.default_target_elevation
```

After the first delivery:

```python
required_kind = self.delivered_kind
preferred_kind = None
preferred_elevation = None
```

Other important inputs:

```python
max_age_s=CONFIG.visible_max_age_s
marker_pitch_high_deg=CONFIG.marker_pitch_high_deg
marker_pitch_low_deg=CONFIG.marker_pitch_low_deg
exclude_ids=self.delivered_ids
```

The selection timeout defaults to `0.8 s` unless `CONFIG.select_timeout_s` is defined.

### Internal sequence

1. Collect fresh visible target candidates:
   - Helper: `get_visible_targets` from the perception layer.

2. Remove ineligible candidates:
   - Apply required kind when one is set.
   - Exclude IDs in `delivered_ids`.

3. Determine target elevation where required:
   - File: `skills/perception/marker_elevation.py`

4. Apply selection preferences in order:
   - preferred elevation;
   - preferred kind;
   - closest remaining target.

5. Store the selected target, kind, elevation, and selected servo methodology.

**On success**

The state machine stores:

```python
self.selected_target
self.selected_kind
self.selected_servo_method
```

then:

```text
SELECT_TARGET -> SEEK_AND_COLLECT (behaviors/approach_object.py)
```

**On failure**

```text
SELECT_TARGET -> GLOBAL_OBJECT_SEARCH (behaviors/global_object_search.py)
```

---

## 9. SEEK_AND_COLLECT / APPROACH OBJECT

**State:** `RobotState.SEEK_AND_COLLECT`

**Primary file called:** `behaviors/approach_object.py`

**Class:** `ApproachObject`

Construction:

```python
ApproachObject(
    pose_bearing_allowed=(
        CONFIG.environment == "simulation"
    )
)
```

Start inputs:

```python
config=CONFIG
selected_target=self.selected_target
selected_kind=self.selected_kind
selected_servo_method=self.selected_servo_method
```

`ApproachObject` therefore executes the target decision already made by `SelectTarget`.

Important subordinate files used by the approach behavior include:

```text
skills/navigation/approach_target_servo.py
skills/navigation/approach_target.py
skills/navigation/align_to_target.py
skills/perception/track_object.py
navigation/height_model.py
policies/vision_grace_period.py
```

The normal Stage 2 path uses `skills/navigation/approach_target_servo.py`.

If Stage 2 perception becomes unavailable, `approach_object.py` may fall back to its Stage 1/dead-reckoning approach path.

### Internal sequence

1. Accept and lock the target already selected by `SelectTarget`:
   - Target tracking: `skills/perception/track_object.py`

2. Maintain visibility and target geometry:
   - Vision-loss policy: `policies/vision_grace_period.py`
   - Height model: `navigation/height_model.py`

3. Run the selected Stage 2 servo approach when available:
   - File: `skills/navigation/approach_target_servo.py`
   - Default methodology: `target_bearing`
   - Optional methodology: `pose_bearing`

4. If the current navigation policy falls back to Stage 1:
   - File: `skills/navigation/approach_target.py`

5. On approach completion, expose the pickup handoff geometry:
   - `final_distance_mm`
   - `final_bearing_deg`
   - `target_is_high`
   - `acquired_id`

**On success**

The state machine records:

```python
self.pending_pickup_id
self.pending_pickup_kind
self.pickup_distance_mm
self.pickup_bearing_deg
self.pickup_target_is_high
```

and clears:

```python
self.selected_target
self.selected_kind
self.selected_servo_method
```

then:

```text
SEEK_AND_COLLECT -> PICKUP_OBJECT (behaviors/pickup_object.py)
```

**On failure**

The selected target is abandoned:

```python
self.selected_target = None
self.selected_kind = None
```

then:

```text
SEEK_AND_COLLECT -> SELECT_TARGET (skills/perception/select_target.py)
```

The source currently notes that this is temporary until the intended target-reacquisition route is established.

---

## 10. PICKUP_OBJECT

**State:** `RobotState.PICKUP_OBJECT`

**Primary file called:** `behaviors/pickup_object.py`

**Class:** `PickupObject`

Purpose: perform the final pickup after `ApproachObject` reaches the pickup handoff geometry.

Start inputs:

```python
config=CONFIG
lvl2=controller.lvl2
motion_backend=controller.motion_backend
distance_mm=self.pickup_distance_mm
bearing_deg=self.pickup_bearing_deg
target_is_high=self.pickup_target_is_high
```

### Internal sequence

1. For a high target, move the lift to the middle position:
   - Primitive: `LiftMiddle`
   - File: `primitives/manipulation/liftmiddle.py`

2. Perform final target alignment:
   - File: `skills/navigation/align_to_target.py`

3. Perform the blind final drive:
   - Config point: `FINAL_APPROACH_MARKER_PUSH`
   - Drive primitive: `primitives/motion/drive.py`

4. Grasp the object:
   - File: `skills/manipulation/grasp_object.py`

5. Verify the grip:
   - File: `skills/manipulation/verify_grip.py`

**On success**

```python
self.last_collected_id = self.pending_pickup_id
```

The pending pickup ID is cleared, then:

```text
PICKUP_OBJECT -> POST_PICKUP_REALIGN (behaviors/post_pickup_realign.py)
```

**On failure**

```python
self.pending_pickup_id = None
self.pending_pickup_kind = None
```

then:

```text
PICKUP_OBJECT -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

---

## 11. POST_PICKUP_REALIGN

**State:** `RobotState.POST_PICKUP_REALIGN`

**Primary file called:** `behaviors/post_pickup_realign.py`

**Class:** `PostPickupRealign`

Current internal sequence:

```text
REVERSE
    ->
LIFT_CARRY
    ->
ROTATE
    ->
DONE
```

### Internal sequence

1. Reverse away from the picked-up object:
   - Config point: `POST_PICKUP_REVERSE_MM`
   - Drive primitive: `primitives/motion/drive.py`

2. Move the lift to the carry position:
   - Primitive: `LiftCarry`
   - File: `primitives/manipulation/liftcarry.py`

3. Rotate to the configured post-pickup heading:
   - Config point: `POST_PICKUP_ROTATE_DEG`
   - Rotate primitive: `primitives/motion/rotate.py`

**On success**

```python
self.recover_localisation_success_state = RobotState.RETURN_TO_BASE
```

then:

```text
POST_PICKUP_REALIGN -> RECOVER_LOCALISATION (behaviors/recover_localisation.py)
```

Successful recovery continues to:

```text
RECOVER_LOCALISATION -> RETURN_TO_BASE (behaviors/return_to_base.py)
```

**On failure**

The same recovery route is used:

```text
POST_PICKUP_REALIGN -> RECOVER_LOCALISATION (behaviors/recover_localisation.py)
```

with success from localisation recovery intended to continue to `RETURN_TO_BASE`.

---

## 12. RECOVER_LOCALISATION

**State:** `RobotState.RECOVER_LOCALISATION`

**Primary file called:** `behaviors/recover_localisation.py`

**Class:** `RecoverLocalisation`

Purpose: rotate in fixed increments until a valid localisation pose is recovered.

Main configuration:

```text
CONFIG.recover_step_deg
CONFIG.recover_max_sweep_deg
CONFIG.recover_settle_time
```

Success condition:

```python
localisation.has_pose()
```

### Internal sequence

1. Rotate one recovery step:
   - Config point: `RECOVER_STEP_DEG`
   - Rotate primitive: `primitives/motion/rotate.py`

2. Wait for the configured settle time:
   - Config point: `RECOVER_SETTLE_TIME`

3. Check whether localisation has a valid pose:
   - `localisation.has_pose()`

4. If no pose is available, repeat the rotate / settle / check sequence.

5. Stop with failure after the configured maximum sweep:
   - Config point: `RECOVER_MAX_SWEEP_DEG`

**On success**

The state machine goes to:

```python
self.recover_localisation_success_state
```

if set; otherwise the default is `RETURN_TO_BASE`.

Typical success routes are:

```text
RECOVER_LOCALISATION -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

or:

```text
RECOVER_LOCALISATION -> RETURN_TO_BASE (behaviors/return_to_base.py)
```

**On failure**

Current code retries localisation recovery:

```text
RECOVER_LOCALISATION -> RECOVER_LOCALISATION (behaviors/recover_localisation.py)
```

The failed behavior is discarded and a new recovery behavior is created on the next update.

---

## 13. RETURN_TO_BASE

**State:** `RobotState.RETURN_TO_BASE`

**Primary file called:** `behaviors/return_to_base.py`

**Class:** `ReturnToBase`

Match-zone input uses the canonical Pi-side I/O API:

```python
controller.io.usb["match_zone"].value
```

The application does not need to know whether the backend resolved that value from fixed configuration, USB media, SR hardware/API, or simulation.

Start inputs:

```python
config=CONFIG
match_zone=controller.io.usb["match_zone"].value
```

Update inputs:

```python
lvl2=controller.lvl2
motion_backend=controller.motion_backend
perception=controller.perception
delivered_ids=self.delivered_ids
arena_observations=controller.latest_arena_observations
observation_timestamp=controller.latest_arena_observation_timestamp
```

Important subordinate files:

```text
skills/navigation/return_to_base_servo.py
config/arena.py
```

`config/arena.py` supplies the zone-dependent guide routes.

When servoing is unavailable, `return_to_base.py` also contains a Stage 1 dead-reckoning fallback.

### Internal sequence

1. Read the match zone through the canonical I/O interface:
   - `controller.io.usb["match_zone"].value`

2. Build the return guide route for that zone:
   - File: `config/arena.py`
   - Function: `return_guide_routes(...)`

3. If servoing is enabled, use Stage 2 vision navigation:
   - File: `skills/navigation/return_to_base_servo.py`

4. If servoing is not enabled, use the Stage 1 dead-reckoning fallback:
   - Primitive: `Drive`
   - File: `primitives/motion/drive.py`
   - Current fallback distance: `STAGE1_RETURN_DISTANCE_MM`

5. On successful Stage 2 arrival, retain the reported `arrival_side` for drop-off.

**On success**

The behavior's arrival side is stored:

```python
self.return_arrival_side
```

then:

```text
RETURN_TO_BASE -> DROPOFF_OBJECT (behaviors/dropoff_object.py)
```

**On failure**

```text
RETURN_TO_BASE -> RECOVER_LOCALISATION (behaviors/recover_localisation.py)
```

No explicit success destination is set here, so `RecoverLocalisation` uses its default destination:

```text
RECOVER_LOCALISATION -> RETURN_TO_BASE (behaviors/return_to_base.py)
```

forming:

```text
RETURN_TO_BASE
    failure
       |
       v
RECOVER_LOCALISATION
    success
       |
       v
RETURN_TO_BASE
```

---

## 14. DROPOFF_OBJECT

**State:** `RobotState.DROPOFF_OBJECT`

**Primary file called:** `behaviors/dropoff_object.py`

**Class:** `DropoffObject`

Start inputs:

```python
config=CONFIG
match_zone=controller.io.usb["match_zone"].value
arrival_side=self.return_arrival_side
delivered_target_id=self.last_collected_id
delivered_ids=self.delivered_ids
```

Update inputs:

```python
lvl2=controller.lvl2
perception=controller.perception
motion_backend=controller.motion_backend
arena_observations=controller.latest_arena_observations
observation_timestamp=controller.latest_arena_observation_timestamp
```

Purpose: perform the delivery/drop-off action and report which object ID was successfully delivered.

### Internal sequence

1. Receive the match zone, arrival side, delivered target ID, and already delivered IDs.

2. If `arrival_side` is `left` or `right`, enter the Stage 2 placement planning step:
   - File: `behaviors/dropoff_object.py`
   - Current placement geometry is still a placeholder and proceeds to release.

3. Release the object:
   - Primitive: `Release`
   - File: `primitives/manipulation/release.py`

4. Report the delivered target ID to `auto_sr2026_stage1.py`.

**On success**

The delivered ID comes from the behavior if available; otherwise the state machine falls back to:

```python
self.last_collected_id
```

The ID is added to:

```python
self.delivered_ids
```

For the first successful delivery only:

```python
self.delivered_kind = self.pending_pickup_kind
```

Subsequent search and selection then require that same kind.

Then:

```text
DROPOFF_OBJECT -> POST_DROPOFF_REALIGN (behaviors/post_dropoff_realign.py)
```

**On failure**

```text
DROPOFF_OBJECT -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

---

## 15. POST_DROPOFF_REALIGN

**State:** `RobotState.POST_DROPOFF_REALIGN`

**Primary file called:** `behaviors/post_dropoff_realign.py`

**Class:** `PostDropoffRealign`

Purpose: move/rotate away from the delivery position so the robot is ready to search for another object.

### Internal sequence

1. Reverse away from the drop-off position:
   - Config point: `POST_DROPOFF_REVERSE_MM`
   - Drive primitive: `primitives/motion/drive.py`

2. Lower the lift:
   - Primitive: `LiftDown`
   - File: `primitives/manipulation/liftdown.py`

3. Rotate to the configured post-dropoff heading:
   - Config point: `POST_DROPOFF_ROTATE_DEG`
   - Rotate primitive: `primitives/motion/rotate.py`

**On success**

```text
POST_DROPOFF_REALIGN -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

**On failure**

Current policy deliberately continues anyway:

```text
POST_DROPOFF_REALIGN -> PREPARE_SEARCH (skills/manipulation/prepare_search.py)
```

So both results restart the object-search cycle.

---
# 16. Strategy memory carried between states

| Variable | Purpose |
|---|---|
| `delivered_ids` | IDs already successfully delivered; excluded from future search/selection |
| `last_collected_id` | Most recently successfully picked-up object |
| `delivered_kind` | Kind established by the first successful delivery; later targets must match |
| `pending_pickup_kind` | Kind being carried through approach/pickup/drop-off |
| `pending_pickup_id` | Object that completed approach and is awaiting pickup |
| `pickup_distance_mm` | Final approach distance handed to `PickupObject` |
| `pickup_bearing_deg` | Final approach bearing handed to `PickupObject` |
| `pickup_target_is_high` | High/low classification handed to `PickupObject` |
| `selected_target` | Target chosen by `SelectTarget` |
| `selected_kind` | Kind of the selected target |
| `selected_servo_method` | Target-dependent approach methodology chosen by `SelectTarget` |
| `return_arrival_side` | Left/right arrival side reported by `ReturnToBase` |
| `recover_localisation_success_state` | State to enter after successful localisation recovery |

---

# 17. Success/failure routing summary

| State | Main file | Success | Failure |
|---|---|---|---|
| `SCRIPTED_START` | `behaviors/scripted_start.py` | `SEEK_AND_COLLECT (behaviors/approach_object.py)` | `SEEK_AND_COLLECT (behaviors/approach_object.py)` |
| `INIT_ESCAPE` | `behaviors/init_escape.py` | `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` | No explicit route (`autonomous/auto_sr2026_stage1.py`) |
| `PREPARE_SEARCH` | `skills/manipulation/prepare_search.py` | `GLOBAL_OBJECT_SEARCH (behaviors/global_object_search.py)` | No transition; routing TBD (`autonomous/auto_sr2026_stage1.py`) |
| `GLOBAL_OBJECT_SEARCH` | `behaviors/global_object_search.py` | `SELECT_TARGET (skills/perception/select_target.py)` | `RECOVER_LOCALISATION (behaviors/recover_localisation.py)`, then `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` |
| `SELECT_TARGET` | `skills/perception/select_target.py` | `SEEK_AND_COLLECT (behaviors/approach_object.py)` | `GLOBAL_OBJECT_SEARCH (behaviors/global_object_search.py)` |
| `SEEK_AND_COLLECT` | `behaviors/approach_object.py` | `PICKUP_OBJECT (behaviors/pickup_object.py)` | `SELECT_TARGET (skills/perception/select_target.py)` |
| `PICKUP_OBJECT` | `behaviors/pickup_object.py` | `POST_PICKUP_REALIGN (behaviors/post_pickup_realign.py)` | `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` |
| `POST_PICKUP_REALIGN` | `behaviors/post_pickup_realign.py` | `RECOVER_LOCALISATION (behaviors/recover_localisation.py)`, then `RETURN_TO_BASE (behaviors/return_to_base.py)` | Same recovery route |
| `RECOVER_LOCALISATION` | `behaviors/recover_localisation.py` | Stored destination: `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` or `RETURN_TO_BASE (behaviors/return_to_base.py)` | Retry `RECOVER_LOCALISATION (behaviors/recover_localisation.py)` |
| `RETURN_TO_BASE` | `behaviors/return_to_base.py` | `DROPOFF_OBJECT (behaviors/dropoff_object.py)` | `RECOVER_LOCALISATION (behaviors/recover_localisation.py)`, then retry `RETURN_TO_BASE (behaviors/return_to_base.py)` |
| `DROPOFF_OBJECT` | `behaviors/dropoff_object.py` | `POST_DROPOFF_REALIGN (behaviors/post_dropoff_realign.py)` | `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` |
| `POST_DROPOFF_REALIGN` | `behaviors/post_dropoff_realign.py` | `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` | `PREPARE_SEARCH (skills/manipulation/prepare_search.py)` |

---
# 18. Main recovery loops

### No target selected

```text
SELECT_TARGET
    failure
       |
       v
GLOBAL_OBJECT_SEARCH
       |
       +---- success ----> SELECT_TARGET
```

### Global search fails

```text
GLOBAL_OBJECT_SEARCH
    failure
       |
       v
RECOVER_LOCALISATION
    success
       |
       v
PREPARE_SEARCH
```

### Approach fails or loses target

```text
APPROACH_OBJECT
    failure
       |
       v
SELECT_TARGET
```

### Pickup fails

```text
PICKUP_OBJECT
    failure
       |
       v
PREPARE_SEARCH
```

### Return-to-base fails

```text
RETURN_TO_BASE
    failure
       |
       v
RECOVER_LOCALISATION
    success
       |
       v
RETURN_TO_BASE
```

### Drop-off fails

```text
DROPOFF_OBJECT
    failure
       |
       v
PREPARE_SEARCH
```

---

# 19. Architecture boundaries

```text
GlobalObjectSearch
    finds whether eligible object candidates exist

SelectTarget
    chooses the target
    chooses the servo methodology

ApproachObject
    owns and tracks that selected target
    executes the requested approach

PickupObject
    performs the final pickup from the approach handoff geometry

ReturnToBase
    navigates the carried object back to the delivery area

DropoffObject
    performs the delivery action

AutoSR2026Stage1
    owns mission-level sequencing and recovery routing
```

Hardware-specific details remain behind the canonical `controller.io.*` interface.

---

# 20. Current routing points worth revisiting

This README documents the code as it currently behaves.

1. `SCRIPTED_START` currently enters `SEEK_AND_COLLECT` directly, bypassing the normal target-selection path.
2. `INIT_ESCAPE` has no explicit autonomous-level failure transition.
3. `PREPARE_SEARCH` failure routing is explicitly marked TBD.
4. `RECOVER_LOCALISATION` failure currently retries indefinitely.
5. `ApproachObject` failure returns to `SELECT_TARGET`; the source currently notes that this is temporary until the intended target-reacquisition route is established.
6. Pose-bearing capability is currently enabled in `AutoSR2026Stage1` only for `CONFIG.environment == "simulation"`.

These are documented here so that the actual state-machine behavior is visible rather than hidden behind implicit retries.

---

# 21. Files directly imported by `auto_sr2026_stage1.py`

```text
state_machine.py

behaviors/init_escape.py
behaviors/approach_object.py
behaviors/pickup_object.py
behaviors/post_pickup_realign.py
behaviors/recover_localisation.py
behaviors/dropoff_object.py
behaviors/post_dropoff_realign.py
behaviors/scripted_start.py
behaviors/return_to_base.py
behaviors/global_object_search.py

skills/manipulation/prepare_search.py
skills/perception/select_target.py

config/__init__.py
config/strategy.py
```

The behavior/skill files above may in turn call lower-level skills, primitives, perception helpers, navigation controllers, and hardware-independent `io.*` interfaces.
