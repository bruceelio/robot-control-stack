import sys
from pathlib import Path
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from skills.perception.track_object import TrackObject


def det(id, kind="cone", bearing=10.0, distance=500.0, t=None):
    return {
        "id": id,
        "kind": kind,
        "bearing": bearing,
        "distance": distance,
        "last_seen_s": time.time() if t is None else t,
    }

trk = TrackObject(kind="cone")

now = time.time()
print(
    trk.update(
        perception_objects={"cone": {3: det(3, t=now)}},
        now_s=now,
        locked_target_id=3,
        kind="cone",
    )
)

# disappear for 0.1s
now2 = now + 0.1
print(
    trk.update(
        perception_objects={"cone": {}},
        now_s=now2,
        locked_target_id=3,
        kind="cone",
    )
)

# reappear
now3 = now2 + 0.1
print(
    trk.update(
        perception_objects={"cone": {3: det(3, t=now3)}},
        now_s=now3,
        locked_target_id=3,
        kind="cone",
    )
)
