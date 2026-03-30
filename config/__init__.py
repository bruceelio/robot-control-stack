# config/__init__.py

from dataclasses import asdict
from importlib import import_module
from pprint import pprint

from .schema import resolve
from . import arena
from . import strategy


profile_module = import_module(
    f"{__name__}.profiles.{strategy.ROBOT_PROFILE.value}"
)

CONFIG = resolve(
    arena=arena,
    profile=profile_module,
    strategy=strategy,
)

print("\n=== RESOLVED CONFIGURATION ===")
pprint(asdict(CONFIG), sort_dicts=False)
print("=== END CONFIGURATION ===\n")
