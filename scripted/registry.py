# scripted/registry.py

from config.strategy import StartupScript
from scripted.programs.script_basic_grab import ScriptBasicGrab
from scripted.programs.script_acidic_grab import ScriptAcidicGrab  # NEW
from scripted.programs.script_motion_test import ScriptMotionTest

SCRIPT_REGISTRY = {
    StartupScript.BASIC_GRAB: ScriptBasicGrab,
    StartupScript.ACIDIC_GRAB: ScriptAcidicGrab,  # NEW
    StartupScript.MOTION_TEST: ScriptMotionTest,
}

