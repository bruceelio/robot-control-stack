import sys
from pathlib import Path
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from behaviors.acquire_object import AcquireObject

class P:
    def __init__(self, objects_by_kind):
        # The simulator pipeline expects perception.objects[kind] to be a dict-like memory
        self.objects = objects_by_kind


class DummyMB:
    def rotate(self, angle_deg: float):
        # SearchRotate -> primitives.motion.rotate.Rotate calls this
        print(f"[DummyMB] rotate({angle_deg=})")

    def drive(self, distance_mm: float):
        # You will hit this once ApproachTarget starts driving
        print(f"[DummyMB] drive({distance_mm=})")

    def stop(self):
        # Some primitives/skills call stop defensively
        print("[DummyMB] stop()")


class DummyLoc:
    pass

a = AcquireObject()

class Cfg:
    default_target_kind = "cone"
    vision_loss_timeout_s = 0.5
    vision_grace_period_s = 0.2
    min_rotate_deg = 5.0
    max_rotate_deg = 45.0

cfg = Cfg()

a.start(config=cfg, kind="cone")

# single tick with no detections
st = a.update(
    lvl2=None,
    perception=P({"cone": {}}),  # empty memory for this kind
    localisation=DummyLoc(),
    motion_backend=DummyMB(),
)

print("status:", st, "phase:", a.phase, "locked:", a.locked_target_id)
# Fake one detection in the expected memory shape:
now = time.time()
det = {"id": 3, "kind": "cone", "bearing": 10.0, "distance": 500.0, "last_seen_s": now}

st = a.update(
    lvl2=None,
    perception=P({"cone": {3: det}}),
    localisation=DummyLoc(),
    motion_backend=DummyMB(),
)
print("status:", st, "phase:", a.phase, "locked:", a.locked_target_id)

