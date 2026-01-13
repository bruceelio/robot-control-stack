# ============================
# Test framework for robot I/O, safety, and motion
# Works in SR simulator and on real hardware
# ============================

# ------------------------------------------------------------------
# Test registry
# ------------------------------------------------------------------

TESTS = {}


def register_test(
    *,
    name=None,
    category="general",
    enabled=True,
    requires_robot=True,
):
    """
    Decorator to register a test.

    enabled=True   -> test is runnable
    enabled=False  -> test is skipped
    requires_robot -> False for pure logic / virtual tests
    """
    def decorator(fn):
        test_name = name or fn.__name__
        TESTS[test_name] = {
            "func": fn,
            "category": category,
            "enabled": enabled,
            "requires_robot": requires_robot,
        }
        return fn
    return decorator


# ------------------------------------------------------------------
# Test runner
# ------------------------------------------------------------------

def run_tests(
    robot=None,
    *,
    only=None,
    category=None,
):
    """
    Run registered tests.

    only="test_name"     -> run a single test
    category="hal"        -> run tests in category
    """
    print("\n=== TEST RUN START ===")

    for name, meta in TESTS.items():
        if not meta["enabled"]:
            continue
        if only and name != only:
            continue
        if category and meta["category"] != category:
            continue

        print(f"\n>>> RUNNING TEST: {name}")
        try:
            if meta["requires_robot"]:
                if robot is None:
                    raise RuntimeError("Robot instance required")
                meta["func"](robot)
            else:
                meta["func"]()
        except Exception as e:
            print(f"!!! TEST FAILED: {name}")
            print(f"    {e}")

    print("\n=== TEST RUN COMPLETE ===\n")


# ------------------------------------------------------------------
# I/O TESTS (LEVEL 1)
# ------------------------------------------------------------------

@register_test(category="hal", enabled=True)
def test_digital_inputs(robot):
    """Reads all DI_* canonicals"""
    from hal.pinmap import canonical_to_pin

    print("Testing digital inputs...")
    found = False

    for name, pin in canonical_to_pin.items():
        if str(name).startswith("DI_") and pin is not None:
            found = True
            try:
                value = robot.arduino.pins[pin].digital_read()
                print(f"{name}: pin {pin} = {value}")
            except Exception as e:
                print(f"{name}: read failed ({e})")

    if not found:
        print("No digital inputs defined.")


@register_test(category="hal", enabled=True)
def test_analog_inputs(robot):
    """Reads all AI_* canonicals"""
    from hal.pinmap import canonical_to_pin

    print("Testing analog inputs...")
    found = False

    for name, pin in canonical_to_pin.items():
        if str(name).startswith("AI_") and pin is not None:
            found = True
            try:
                value = robot.arduino.pins[pin].analog_read()
                print(f"{name}: pin {pin} = {value}")
            except Exception as e:
                print(f"{name}: read failed ({e})")

    if not found:
        print("No analog inputs defined.")


@register_test(category="hal", enabled=False)
def test_digital_outputs(robot):
    """Toggles all DO_* canonicals"""
    from hal.pinmap import canonical_to_pin

    print("Testing digital outputs...")

    for name, pin in canonical_to_pin.items():
        if str(name).startswith("DO_") and pin is not None:
            print(f"Toggling {name}")
            try:
                if isinstance(pin, str):
                    print(f"  simulated output: {pin}")
                else:
                    robot.arduino.pins[pin].digital_write(True)
                    robot.sleep(0.2)
                    robot.arduino.pins[pin].digital_write(False)
            except Exception as e:
                print(f"{name}: write failed ({e})")


# ------------------------------------------------------------------
# LEVEL 2 / MOTION TESTS
# ------------------------------------------------------------------

@register_test(category="motion", enabled=False)
def test_level2_drive(robot):
    """Visible movement test in simulator"""
    from level2_canonical import Level2

    print("Testing Level 2 DRIVE...")
    lvl2 = Level2(robot)

    lvl2.DRIVE(0.5, 0.5, duration=1.0)
    lvl2.DRIVE(-0.5, -0.5, duration=1.0)
    lvl2.ROTATE(90)
    lvl2.ROTATE(-90)


# ------------------------------------------------------------------
# SAFETY / VIRTUAL TESTS (NO ROBOT REQUIRED)
# ------------------------------------------------------------------

@register_test(category="safety", enabled=True, requires_robot=False)
def test_virtual_front_bumper():
    """Tests SI_BUMPER_FRONT logic"""
    from hal.canonical import (
        DI_BUMPER_FRONT_LEFT,
        DI_BUMPER_FRONT_RIGHT,
        SI_BUMPER_FRONT,
        update_virtual_bumpers,
    )

    print("Testing SI_BUMPER_FRONT...")

    def mock_read(di):
        return di in (DI_BUMPER_FRONT_LEFT, DI_BUMPER_FRONT_RIGHT)

    bumpers = update_virtual_bumpers(mock_read)
    print("SI_BUMPER_FRONT =", bumpers[SI_BUMPER_FRONT])
