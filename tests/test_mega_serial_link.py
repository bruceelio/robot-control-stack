# tests/test_mega_serial_link.py

from __future__ import annotations

import time

from tests.registry import register_test

try:
    from config import CONFIG
except Exception:
    CONFIG = None

from tests.mega_serial_client import MegaSerialClient, MegaSerialConfig


def _mega_config() -> MegaSerialConfig:
    if CONFIG is None:
        return MegaSerialConfig()

    return MegaSerialConfig(
        port=getattr(CONFIG, "mega_port", "/dev/ttyACM0"),
        baud=getattr(CONFIG, "mega_baud", 115200),
        timeout=getattr(CONFIG, "mega_timeout", 1.0),
        open_delay_s=getattr(CONFIG, "mega_open_delay_s", 2.0),
    )


def _expect_prefix(actual: str, prefix: str) -> None:
    assert actual.startswith(prefix), f"expected prefix {prefix!r}, got {actual!r}"


def _log(msg: str) -> None:
    print(f"[MEGA LINK] {msg}")


def _sleep_with_heartbeat(
    mega: MegaSerialClient,
    *,
    start_seq: int,
    duration_s: float,
    step_s: float = 0.1,
) -> int:
    seq = start_seq
    end_t = time.time() + duration_s

    while time.time() < end_t:
        resp = mega.heartbeat(seq)
        _expect_prefix(resp, f"OK HB {seq}")
        seq += 1
        time.sleep(step_s)

    return seq


@register_test(category="hal", enabled=True, requires_robot=False)
def test_mega_serial_hello():
    cfg = _mega_config()
    _log(f"opening {cfg.port} @ {cfg.baud}")

    with MegaSerialClient(cfg) as mega:
        resp = mega.hello()
        _log(f"HELLO -> {resp}")
        _expect_prefix(resp, "ID ")


@register_test(category="hal", enabled=True, requires_robot=False)
def test_mega_serial_auto_basic():
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")
        _expect_prefix(mega.mode_auto(), "OK MODE AUTO")
        _expect_prefix(mega.heartbeat(1), "OK HB 1")
        _expect_prefix(mega.stop(), "OK STOP")
        _expect_prefix(mega.mode_teleop(), "OK MODE TELEOP")


@register_test(category="hal", enabled=True, requires_robot=False)
def test_mega_serial_drive_and_grip():
    """
    Hardware-in-the-loop motion test using only hardware-native API.
    """
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")
        _expect_prefix(mega.mode_auto(), "OK MODE AUTO")

        try:
            seq = 1

            # LEFT via RoboClaw A M1
            _log("LEFT motor forward")
            _expect_prefix(mega.link_18_19("M1", 0.4), "OK LINK 18 19 M1")
            _expect_prefix(mega.link_18_19("M2", 0.0), "OK LINK 18 19 M2")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=2.0)

            _expect_prefix(mega.stop(), "OK STOP")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=0.5)

            # RIGHT via RoboClaw A M2
            _log("RIGHT motor forward")
            _expect_prefix(mega.link_18_19("M1", 0.0), "OK LINK 18 19 M1")
            _expect_prefix(mega.link_18_19("M2", 0.4), "OK LINK 18 19 M2")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=2.0)

            _expect_prefix(mega.stop(), "OK STOP")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=0.5)

            # BOTH
            _log("BOTH motors forward")
            _expect_prefix(mega.link_18_19("M1", 0.4), "OK LINK 18 19 M1")
            _expect_prefix(mega.link_18_19("M2", 0.4), "OK LINK 18 19 M2")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=2.0)

            _expect_prefix(mega.stop(), "OK STOP")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=0.5)

            # GRIP CLOSE
            _log("GRIP close")
            _expect_prefix(mega.group_write(11, 1.0, 13, -1.0), "OK GROUP_WRITE 11 13")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=1.0)

            # GRIP OPEN
            _log("GRIP open")
            _expect_prefix(mega.group_write(11, -1.0, 13, 1.0), "OK GROUP_WRITE 11 13")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=1.0)

        finally:
            try:
                mega.stop()
            except Exception:
                pass
            try:
                mega.mode_teleop()
            except Exception:
                pass


@register_test(category="hal", enabled=False, requires_robot=False)
def test_mega_serial_lift():
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")
        _expect_prefix(mega.mode_auto(), "OK MODE AUTO")

        try:
            seq = 1

            _log("LIFT down")
            _expect_prefix(mega.servo_write(12, -1.0), "OK SERVO_WRITE 12")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=1.0)

            _log("LIFT up")
            _expect_prefix(mega.servo_write(12, 1.0), "OK SERVO_WRITE 12")
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=1.0)

        finally:
            try:
                mega.mode_teleop()
            except Exception:
                pass


@register_test(category="hal", enabled=False, requires_robot=False)
def test_mega_serial_hbridge():
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")
        _expect_prefix(mega.mode_auto(), "OK MODE AUTO")

        try:
            seq = 1

            _log("SHOOTER motor forward")
            _expect_prefix(
                mega.hbridge_write(ina=30, inb=31, en_diag=32, pwm=4, value=0.5),
                "OK HBRIDGE_WRITE 30 31 32 4",
            )
            seq = _sleep_with_heartbeat(mega, start_seq=seq, duration_s=1.0)

            _expect_prefix(mega.stop(), "OK STOP")

        finally:
            try:
                mega.mode_teleop()
            except Exception:
                pass


@register_test(category="hal", enabled=False, requires_robot=False)
def test_mega_serial_reads():
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")

        di = mega.digital_read(26)
        _log(f"READ DI 26 -> {di}")
        _expect_prefix(di, "OK DI ")

        ai = mega.analog_read("A1")
        _log(f"READ AI A1 -> {ai}")
        _expect_prefix(ai, "OK AI ")

        batt = mega.read_battery("voltage")
        _log(f"READ BATTERY voltage -> {batt}")
        _expect_prefix(batt, "OK BATTERY ")

        quad = mega.quad_read(22, 23)
        _log(f"READ QUAD 22 23 -> {quad}")
        _expect_prefix(quad, "OK QUAD ")

        rng = mega.range_read(2, 3)
        _log(f"READ RANGE 2 3 -> {rng}")
        _expect_prefix(rng, "OK RANGE ")


@register_test(category="hal", enabled=True, requires_robot=False)
def test_mega_serial_heartbeat_timeout():
    cfg = _mega_config()

    with MegaSerialClient(cfg) as mega:
        _expect_prefix(mega.hello(), "ID ")
        _expect_prefix(mega.mode_auto(), "OK MODE AUTO")
        _expect_prefix(mega.heartbeat(1), "OK HB 1")

        _log("commanding grip close, then intentionally withholding heartbeat")
        _expect_prefix(mega.group_write(11, 1.0, 13, -1.0), "OK GROUP_WRITE 11 13")

        _log("waiting > heartbeat timeout so Mega should fall back out of AUTO")
        time.sleep(1.0)

        try:
            mega.mode_teleop()
        except Exception:
            pass
