from tests.registry import register_test


@register_test(category="hal", enabled=True)
def test_digital_inputs(robot):
    """Reads all DI_* canonicals"""
    from legacy.hal.pinmap import canonical_to_pin

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
    from legacy.hal.pinmap import canonical_to_pin

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
    from legacy.hal.pinmap import canonical_to_pin

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
