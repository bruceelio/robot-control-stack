### 1) Localisation is an “estimator + arbitrator”

Localisation should not “compute pose” in one monolithic function. Instead it:

* collects **pose observations** from one or more methods
* picks the best one (or fuses later)
* maintains the current `Pose` state with validity + staleness

So you get a single owner of truth, but many suppliers.

---

## Recommended folder layout

```
localisation/
  __init__.py
  localisation.py          # owns state: current pose, validity, staleness, apply_motion
  pose_types.py            # Pose, PoseObservation, confidence model
  providers/
    __init__.py
    base.py                # PoseProvider interface
    cam1_markers2.py       # current method (1 camera, 2 arena markers)
    cam1_markers3.py       # alternate (1 camera, 3+ markers)
    cam2_markers2.py       # later (two cameras, min markers)
    ...                    # add more as needed
  arbitration.py           # priority/scoring logic (simple now, smarter later)
```

This matches your “location.py chooses priority, separate implementations per method” idea, but avoids filename sprawl at the top-level.

---

## The key interface: `PoseObservation`

Each provider returns a `PoseObservation` rather than directly setting pose:

**Fields you’ll want from day one:**

* `position: (x, y)` always when available
* `heading: Optional[float]` (can be `None`)
* `covariance` *or* a simpler `confidence` scalar (start simple)
* `timestamp`
* `source` (e.g., `"cam1_markers2"`)
* `markers_used` (count + ids) for debug

Example semantics (no code dump, just what it represents):

* Provider A (cam1, 2 markers) might return:

  * position OK, heading unknown, confidence 0.6
* Provider B (cam1, 3 markers) might return:

  * position OK, heading weak, confidence 0.75
* Provider C (cam2, 4+ markers) might return:

  * position+heading strong, confidence 0.9

Localisation then decides which to accept.

---

## How arbitration should work (simple now, scalable later)

Start with a **scoring function**, not a rigid “priority order”. It’ll save you pain later.

A good early scoring heuristic:

* base score by provider type (e.g., 2 cameras tends to be better)
* * bonus for more markers used (up to a cap)
* * bonus if heading is present
* − penalty if observation is old
* − penalty if it jumps too far from last pose (sanity gating)

So you can keep *one* configuration table like:

```text
provider_weight:
  cam1_markers2: 0.6
  cam1_markers3: 0.75
  cam2_markers2: 0.8
  cam2_markers3plus: 0.9
```

Then adjust with marker count and freshness.

This is better than “provider A always wins”, because sometimes a “lower tier” provider can be fresh while a “higher tier” one is stale or glitching.

---

## Where the providers get their inputs

A provider should take **raw detections**, not read the camera itself. That keeps it testable and clean.

So the flow becomes:

* `perception` returns: “arena markers seen this frame” (bearing, distance, id, camera_id)
* `localisation` feeds those detections into each provider
* each provider tries to compute a `PoseObservation`
* arbitration picks best and updates pose

This removes the cross-contamination cleanly.

---

## Naming your provider files

Your instinct “location_*camera*_markers*.py” is understandable, but as you add variations it’ll get messy.

I’d name by **capability**, not exact inputs, and include camera count / marker requirements only when it matters:

* `vision_singlecam_trilateration.py` (2-marker position only)
* `vision_singlecam_multimarker.py` (3+ markers)
* `vision_multicam_fusion.py` (later)

Or if you really want explicitness:

* `cam1_markers2.py`, `cam1_markers3plus.py`, `cam2_markers2plus.py`

That stays readable without becoming absurd.

---

## What this means for your current code (minimal step)

You can implement this without a big refactor by doing just these steps:

1. Create `localisation/providers/cam1_markers2.py` and move your current `estimate_pose` math there.
2. Localisation calls it each tick with “arena marker detections”.
3. Perception stops storing pose; it only provides marker detections and object detections.

Nothing else needs to know *how* pose was computed.

---

## One important future detail: heading is a first-class citizen

Design the `Pose` as:

* `position_valid: bool`
* `heading_valid: bool`

Because your near-term providers may only give position, and it’s crucial not to accidentally treat heading as “0 = valid”.

That single change prevents a lot of downstream weirdness.

---

If you want, I’ll propose the exact *class skeletons* (interfaces only) for:

* `PoseObservation`
* `PoseProvider`
* `Localisation.update_from_vision(detections)`

…so you can implement provider-by-provider without touching behaviors yet.
