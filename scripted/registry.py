# scripted/registry.py

from config.strategy import StartupScript
from scripted.programs.script_basic_grab import ScriptBasicGrab
from scripted.programs.script_acidic_grab import ScriptAcidicGrab  # NEW

SCRIPT_REGISTRY = {
    StartupScript.BASIC_GRAB: ScriptBasicGrab,
    StartupScript.ACIDIC_GRAB: ScriptAcidicGrab,  # NEW
}

