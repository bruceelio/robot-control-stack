# config/__init__.py

from dataclasses import asdict
from pprint import pprint

from .schema import resolve
from . import arena
from .profiles import simulation
from . import strategy

CONFIG = resolve(
    arena=arena,
    profile=simulation,
    strategy=strategy,
)

print("\n=== RESOLVED CONFIGURATION ===")
pprint(asdict(CONFIG), sort_dicts=False)
print("=== END CONFIGURATION ===\n")

