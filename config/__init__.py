# config/__init__.py


from importlib import import_module

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

