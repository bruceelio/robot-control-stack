# tests/test_io_checkout.py
from tests.registry import register_test
from hw_io.resolve import resolve_io
from config import CONFIG
import json
from datetime import datetime


def _io(robot):
    return resolve_io(robot=robot, hardware_profile=CONFIG.hardware_profile)


def _device_log(device: str, *, passed: bool, data=None, note: str = "") -> None:
    """
    Human-readable per-device log block, plus a single JSON line for tooling.
    - device: explicit code name, e.g. "io.cameras()['front']"
    - data: dict/list/primitive returned from IO
    """
    status = "PASS" if passed else "FAIL"
    ts = datetime.now().isoformat(timespec="seconds")

    print("\n" + "=" * 60)
    print(f"[IO][{status}] {device}")
    print(f"timestamp: {ts}")
    if note:
        print(f"note: {note}")

    if data is not None:
        # Pretty display
        if isinstance(data, (dict, list)):
            print("data:")
            print(json.dumps(data, indent=2, sort_keys=True, default=str))
        else:
            print(f"data: {data}")

    # Optional: machine-friendly one-liner (easy to parse later)
    record = {"ts": ts, "device": device, "status": status, "note": note, "data": data}
    print("json:", json.dumps(record, separators=(",", ":"), sort_keys=True, default=str))
    print("=" * 60)


# -------------------------
# Safe read-only tests
# -------------------------

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_camera_front(robot):
    io = _io(robot)
    device = "io.cameras()['front']"

    try:
        cams = io.cameras()
        assert "front" in cams, "front camera not found in io.cameras()"

        markers = cams["front"].see()
        _device_log(
            device,
            passed=True,
            data={"io.cameras()['front'].see()": len(markers)},
            note="see() ok",
        )
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_battery(robot):
    io = _io(robot)
    device = "io.battery()"

    try:
        b = io.battery()
        v = b.get("voltage")
        c = b.get("current")

        _device_log(
            device,
            passed=True,
            data={
                "io.battery()['voltage']": v,
                "io.battery()['current']": c,
            },
        )
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_bumpers(robot):
    io = _io(robot)
    device = "io.bumpers()"

    try:
        b = io.bumpers()
        assert isinstance(b, dict), f"bumpers() must return dict, got {type(b)}: {b}"
        for k in ("fl", "fr", "rl", "rr"):
            assert k in b, f"missing bumper key: {k}"

        data = {f"io.bumpers()['{k}']": b[k] for k in ("fl", "fr", "rl", "rr")}
        _device_log(device, passed=True, data=data)
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_reflectance(robot):
    io = _io(robot)
    device = "io.reflectance()"

    try:
        r = io.reflectance()
        assert isinstance(r, dict), f"reflectance() must return dict, got {type(r)}: {r}"
        for k in ("left", "center", "right"):
            assert k in r, f"missing reflectance key: {k}"

        data = {f"io.reflectance()['{k}']": r[k] for k in ("left", "center", "right")}
        _device_log(device, passed=True, data=data)
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_ultrasonics(robot):
    io = _io(robot)
    device = "io.ultrasonics()"

    try:
        u = io.ultrasonics()
        assert isinstance(u, dict), f"ultrasonics() must return dict, got {type(u)}: {u}"
        for k in ("front", "left", "right", "back"):
            assert k in u, f"missing ultrasonic key: {k}"

        data = {
            f"io.ultrasonics()['{k}']": u[k]
            for k in ("front", "left", "right", "back")
        }
        _device_log(device, passed=True, data=data)
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_servo0_present(robot):
    io = _io(robot)
    device = "io.servos[0]"

    try:
        servos = getattr(io, "servos", None)
        assert servos is not None, "io.servos missing"
        assert len(servos) >= 1, f"io.servos has no channel 0 (len={len(servos)})"

        servo0 = servos[0]

        # Best-effort readback for logging (may not exist on all adapters)
        state = {}
        if hasattr(servo0, "position"):
            try:
                state["io.servos[0].position"] = servo0.position
            except Exception:
                pass

        _device_log(device, passed=True, data={"present": True, **state}, note="servo channel 0 present")
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


# -------------------------
# Actuation tests (opt-in)
# -------------------------
# These are disabled by default for safety on SR1 Real.
# Enable temporarily during bring-up, or run individually via run_tests(only="...")
# enabled=True

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_vacuum_pulse(robot):
    io = _io(robot)
    outs = io.outputs
    device = "io.outputs.set('VACUUM', <bool>)"
    assert outs is not None, "io.outputs missing"

    try:
        outs.set("VACUUM", True)
        io.sleep(0.25)
        outs.set("VACUUM", False)
        io.sleep(0.25)

        _device_log(
            device,
            passed=True,
            data={
                "sequence": [
                    "io.outputs.set('VACUUM', True)",
                    "sleep(0.25)",
                    "io.outputs.set('VACUUM', False)",
                    "sleep(0.25)",
                ]
            },
            note="VACUUM on/off pulse completed",
        )
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_motors_jog(robot):
    """
    Opt-in: small motor jog. Keep very short.
    """
    io = _io(robot)
    motors = io.motors
    device = "io.motors[0].power / io.motors[1].power"
    assert motors is not None, "io.motors missing"

    p = 0.4
    dur = 0.5

    try:
        motors[0].power = p
        motors[1].power = p
        io.sleep(dur)
        motors[0].power = 0
        motors[1].power = 0

        _device_log(
            device,
            passed=True,
            data={
                "io.motors[0].power": [p, 0],
                "io.motors[1].power": [p, 0],
                "duration_s": dur,
            },
            note="short jog completed",
        )
    except Exception as e:
        # best-effort stop on failure
        try:
            motors[0].power = 0
            motors[1].power = 0
        except Exception:
            pass
        _device_log(device, passed=False, note=str(e))
        raise

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_lift_servo0_pulse(robot):
    """
    Opt-in: lift servo pulse using Level2 canonical semantics.
    Assumption (per Level2): lift is io.servos[0], position in [-1, +1], None disables.
    """
    io = _io(robot)
    device = "io.servos[0].position  # lift (-1..+1, None=disable)"

    try:
        servos = getattr(io, "servos", None)
        assert servos is not None, "io.servos missing"
        assert len(servos) >= 1, f"io.servos has no channel 0 (len={len(servos)})"

        servo0 = servos[0]

        # DOWN
        servo0.position = -1
        io.sleep(0.40)

        # UP
        servo0.position = 1
        io.sleep(0.40)

        # DISABLE (optional, but matches Level2)
        servo0.position = None

        _device_log(
            device,
            passed=True,
            data={
                "sequence": [
                    "io.servos[0].position = -1  (DOWN)",
                    "sleep(0.40)",
                    "io.servos[0].position =  1  (UP)",
                    "sleep(0.40)",
                    "io.servos[0].position = None (DISABLE)",
                ]
            },
            note="lift servo pulse completed",
        )
    except Exception as e:
        # Best-effort disable on failure
        try:
            servos = getattr(io, "servos", None)
            if servos and len(servos) >= 1:
                servos[0].position = None
        except Exception:
            pass

        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_buzzer_tones(robot):
    io = _io(robot)
    device = "io.buzzer().buzz(tone, duration, blocking=...)"

    try:
        buz = io.buzzer()
        assert buz is not None, "io.buzzer() not available"

        # 400Hz for 0.2s
        buz.buzz(400, 0.2, blocking=True)
        io.sleep(0.05)

        # If SR Note exists, try a note (optional)
        try:
            from sr.robot3 import Note
            buz.buzz(Note.D6, 0.2, blocking=True)
        except Exception:
            pass

        buz.off()

        _device_log(device, passed=True, data={"sequence": ["buzz(400,0.2,blocking=True)", "buzz(Note.D6,0.2)", "off()"]})
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise



@register_test(category="io", enabled=True, requires_robot=True)
def test_io_kch_led_a_cycle(robot):
    """
    Opt-in: cycles KCH LED A colours if available.
    """
    io = _io(robot)
    kch = io.kch()
    device = "io.kch().set_colour(LED_A, Colour.<...>)"
    assert kch is not None, "io.kch() not available"

    from sr.robot3 import Colour, LED_A

    colours = [Colour.RED, Colour.GREEN, Colour.BLUE, Colour.OFF]

    try:
        for colour in colours:
            kch.set_colour(LED_A, colour)
            io.sleep(1.0)

        _device_log(
            device,
            passed=True,
            data={"LED": "LED_A", "colours": [str(c) for c in colours]},
            note="RED->GREEN->BLUE->OFF",
        )
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise

@register_test(category="io", enabled=True, requires_robot=True)
def test_io_kch_led_b_cycle(robot):
    io = _io(robot)
    device = "io.kch().set_colour(LED_B, Colour.<...>)"

    try:
        kch = io.kch()
        assert kch is not None, "io.kch() not available"

        from sr.robot3 import Colour, LED_B
        colours = [Colour.RED, Colour.GREEN, Colour.BLUE, Colour.OFF]

        for colour in colours:
            kch.set_colour(LED_B, colour)
            io.sleep(0.25)

        _device_log(device, passed=True, data={"LED": "LED_B", "colours": [str(c) for c in colours]})
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise


@register_test(category="io", enabled=True, requires_robot=True)
def test_io_kch_led_c_cycle(robot):
    io = _io(robot)
    device = "io.kch().set_colour(LED_C, Colour.<...>)"

    try:
        kch = io.kch()
        assert kch is not None, "io.kch() not available"

        from sr.robot3 import Colour, LED_C
        colours = [Colour.RED, Colour.GREEN, Colour.BLUE, Colour.OFF]

        for colour in colours:
            kch.set_colour(LED_C, colour)
            io.sleep(0.25)

        _device_log(device, passed=True, data={"LED": "LED_C", "colours": [str(c) for c in colours]})
    except Exception as e:
        _device_log(device, passed=False, note=str(e))
        raise
