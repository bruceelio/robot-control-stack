# ============================
# Hardware test routines for SR Robot + optional extra boards
# ============================

from io_unified import canonical_to_pin  # unified canonical -> pin mapping
from level2_canonical import Level2      # ← NEW


def test_digital_inputs(robot):
    """
    Iterates over all DI_* canonical names and reads their state.
    Prints results to console.
    """
    print("\n=== Testing Digital Inputs ===")
    any_tested = False
    for name, pin in canonical_to_pin.items():
        if str(name).startswith("DI_") and pin is not None:
            any_tested = True
            try:
                value = robot.arduino.pins[pin].digital_read()
                print(f"{name}: pin {pin} = {value}")
            except Exception as e:
                print(f"{name}: pin {pin} read failed: {e}")
    if not any_tested:
        print("No digital input pins found to test.")


def test_analog_inputs(robot):
    """
    Iterates over all AI_* canonical names and reads their state.
    Prints results to console.
    """
    print("\n=== Testing Analog Inputs ===")
    any_tested = False
    for name, pin in canonical_to_pin.items():
        if str(name).startswith("AI_") and pin is not None:
            any_tested = True
            try:
                value = robot.arduino.pins[pin].analog_read()
                print(f"{name}: pin {pin} = {value}")
            except Exception as e:
                print(f"{name}: pin {pin} read failed: {e}")
    if not any_tested:
        print("No analog input pins found to test.")


def test_digital_outputs(robot, test_duration=0.2):
    """
    Iterates over all DO_* canonical names and toggles them on/off.
    Prints results to console.
    """
    print("\n=== Testing Digital Outputs ===")
    any_tested = False
    for name, pin in canonical_to_pin.items():
        if str(name).startswith("DO_") and pin is not None:
            any_tested = True
            try:
                if isinstance(pin, str):
                    print(f"{name}: mapped to '{pin}', simulating toggle ON/OFF")
                    print(f"  {name} -> ON")
                    robot.sleep(test_duration)
                    print(f"  {name} -> OFF")
                else:
                    print(f"Toggling {name} (pin {pin}) ON")
                    robot.arduino.pins[pin].digital_write(True)
                    robot.sleep(test_duration)
                    print(f"Toggling {name} (pin {pin}) OFF")
                    robot.arduino.pins[pin].digital_write(False)
            except Exception as e:
                print(f"{name}: pin {pin} write failed: {e}")
    if not any_tested:
        print("No digital output pins found to test.")


# ============================================================
# NEW: LEVEL 2 TESTS
# ============================================================

def test_level2_drive(robot):
    """
    Visibly drives motors in the SR simulator using Level 2 canonicals.
    """
    print("\n=== Testing Level 2 DRIVE (Simulator Visible) ===")

    lvl2 = Level2(robot)

    print("Driving forward...")
    lvl2.DRIVE(0.5, 0.5, duration=1.5)

    print("Driving backward...")
    lvl2.DRIVE(-0.5, -0.5, duration=1.5)

    print("Rotating clockwise...")
    lvl2.ROTATE(90)

    print("Rotating counter-clockwise...")
    lvl2.ROTATE(-90)

    print("Level 2 DRIVE test complete.")


def run_all_tests(robot):
    """
    Convenience function: run all tests in sequence.
    """
    test_digital_inputs(robot)
    test_analog_inputs(robot)
    test_digital_outputs(robot)

    # Level 2 tests last (they move the robot)
    test_level2_drive(robot)
