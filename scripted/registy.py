# scripted/registry.py

from config.strategy import StartupScript
from scripted.programs.script_basic_grab import ScriptBasicGrab


SCRIPT_REGISTRY = {
    StartupScript.BASIC_GRAB: ScriptBasicGrab,
}
