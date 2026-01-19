# tests/test_io_checkout.py
from tests.registry import register_test
from hw_io.resolve import resolve_io
from config import CONFIG


def _io(robot):
    return resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)


def _tsv(name: str, *, passed: bool, note: str = "") -> None:
    """
    Single-line TSV suitable for copy/paste into Excel.
    Columns: TestName  Result  Notes
    """
    result = "PASS" if passed else "FAIL"
    # Keep it one line for Excel friendliness
    note = (note or "").replace("\t", " ").replace("\n", " ").strip()
    print(f"TSV\t{name}\t{result}\t{note}")


# -------------------------
# Safe read-only tests
# -------------------------

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_camera_front(robot):
    io = _io(robot)
    cams = io.cameras()
    assert "front" in cams, "front camera not found in io.cameras()"
    markers = cams["front"].see()
    print(f"[IO] camera front: {len(markers)} markers")
    _tsv("CAMERA_FRONT_SEE", passed=True, note=f"{len(markers)} markers")


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_battery(robot):
    io = _io(robot)
    b = io.battery()
    v = b.get("voltage")
    c = b.get("current")
    print(f"[IO] battery: voltage={v} current={c}")
    _tsv("BATTERY_VOLTAGE_CURRENT", passed=True, note=f"V={v} I={c}")


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_bumpers(robot):
    io = _io(robot)
    b = io.bumpers()
    assert isinstance(b, dict), f"bumpers() must return dict, got {type(b)}: {b}"
    for k in ("fl", "fr", "rl", "rr"):
        assert k in b, f"missing bumper key: {k}"
    print(f"[IO] bumpers: {b}")
    _tsv("BUMPERS", passed=True, note=str(b))


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_reflectance(robot):
    io = _io(robot)
    r = io.reflectance()
    assert isinstance(r, dict), f"reflectance() must return dict, got {type(r)}: {r}"
    for k in ("left", "center", "right"):
        assert k in r, f"missing reflectance key: {k}"
    print(f"[IO] reflectance: {r}")
    _tsv("REFLECTANCE", passed=True, note=str(r))


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_ultrasonics(robot):
    io = _io(robot)
    u = io.ultrasonics()
    assert isinstance(u, dict), f"ultrasonics() must return dict, got {type(u)}: {u}"
    for k in ("front", "left", "right", "back"):
        assert k in u, f"missing ultrasonic key: {k}"
    print(f"[IO] ultrasonics: {u}")
    _tsv("ULTRASONICS", passed=True, note=str(u))


# -------------------------
# Actuation tests (opt-in)
# -------------------------
# These are disabled by default for safety on SR1 Real.
# Enable temporarily during bring-up, or run individually via run_tests(only="...")

@register_test(category="io", enabled=False, requires_robot=True)
def test_io_vacuum_pulse(robot):
    io = _io(robot)
    outs = io.outputs
    assert outs is not None, "io.outputs missing"

    print("[IO] VACUUM ON")
    outs.set("VACUUM", True)
    io.sleep(0.25)

    print("[IO] VACUUM OFF")
    outs.set("VACUUM", False)
    io.sleep(0.25)

    _tsv("VACUUM_PULSE", passed=True, note="VACUUM on/off pulse completed")


@register_test(category="io", enabled=False, requires_robot=True)
def test_io_motors_jog(robot):
    """
    Opt-in: small motor jog. Keep very short.
    """
    io = _io(robot)
    motors = io.motors
    assert motors is not None, "io.motors missing"

    p = 0.2
    print(f"[IO] motors jog power={p}")
    motors[0].power = p
    motors[1].power = p
    io.sleep(0.20)
    motors[0].power = 0
    motors[1].power = 0

    _tsv("MOTORS_JOG", passed=True, note=f"power={p} duration=0.20s")


@register_test(category="io", enabled=False, requires_robot=True)
def test_io_buzzer_pulse(robot):
    io = _io(robot)
    buz = io.buzzer()
    assert buz is not None, "io.buzzer() not available"

    print("[IO] buzzer on")
    buz.on(duration=0.25)
    io.sleep(0.30)

    print("[IO] buzzer off")
    try:
        buz.off()
    except Exception as e:
        _tsv("BUZZER_PULSE", passed=False, note=f"off failed: {e}")
        raise

    _tsv("BUZZER_PULSE", passed=True, note="on 0.25s then off")


@register_test(category="io", enabled=False, requires_robot=True)
def test_io_kch_led_cycle(robot):
    """
    Opt-in: cycles KCH LED A colours if available.
    """
    io = _io(robot)
    kch = io.kch()
    assert kch is not None, "io.kch() not available"

    from sr.robot3 import Colour, LED_A

    for colour in [Colour.RED, Colour.GREEN, Colour.BLUE, Colour.OFF]:
        print(f"[IO] KCH LED_A -> {colour}")
        kch.set_colour(LED_A, colour)
        io.sleep(0.25)

    _tsv("KCH_LED_A_CYCLE", passed=True, note="RED->GREEN->BLUE->OFF")
