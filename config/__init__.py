# config/__init__.py

from dataclasses import asdict
from pprint import pprint

from .schema import resolve
from . import arena
from .profiles import strategy
from .profiles import simulation  # or sr1 later

CONFIG = resolve(
    arena=arena,
    profile=simulation,
    strategy=strategy,
)

CONFIG.dump()

print("\n=== RESOLVED CONFIGURATION ===")
pprint(asdict(CONFIG), sort_dicts=False)
print("=== END CONFIGURATION ===\n")
