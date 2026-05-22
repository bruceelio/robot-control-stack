# tools/rotate_scan_markers_2WD.py

# python3 -m tools.rotate_scan_markers_2WD

# Operator prompt asks for a signed motor power:
# 0.10  = slow rotation one way
# -0.10 = slow rotation the other way


from __future__ import annotations

import math
import select
import sys
import termios
import time
import tty

from hw_io.bob_bot import BobBotIO


LEFT_MOTOR = "drive_front_left"
RIGHT_MOTOR = "drive_front_right"
CAMERA = "front"
SCAN_INTERVAL_S = 0.25


def key_pressed() -> bool:
    ready, _, _ = select.select([sys.stdin], [], [], 0)
    return bool(ready)


def print_markers(markers) -> None:
    if not markers:
        print("markers: 0")
        return

    print(f"markers: {len(markers)}")

    for marker in markers:
        pos = marker.position
        print(
            f"  id={marker.id} "
            f"distance_mm={pos.distance:.0f} "
            f"bearing_deg={math.degrees(pos.horizontal_angle):.1f} "
            f"vertical_deg={math.degrees(pos.vertical_angle):.1f}"
        )


def set_rotation(io: BobBotIO, power: float) -> None:
    io.motor[LEFT_MOTOR].power = power
    io.motor[RIGHT_MOTOR].power = -power


def stop_drive(io: BobBotIO) -> None:
    io.motor[LEFT_MOTOR].power = 0.0
    io.motor[RIGHT_MOTOR].power = 0.0


def ask_power() -> float:
    print("Enter rotation power as a signed motor power from -1.0 to 1.0.")
    print("Example: 0.10 for slow rotation one way, -0.10 for the opposite way.")
    print("If the robot turns the wrong direction, stop and rerun with the opposite sign.")

    while True:
        raw = input("Rotation power [-1.0..1.0]: ").strip()

        try:
            value = float(raw)
        except ValueError:
            print("Please enter a number, for example 0.10 or -0.10.")
            continue

        if -1.0 <= value <= 1.0:
            return value

        print("Power must be between -1.0 and 1.0.")


def main() -> None:
    io = BobBotIO(robot=None)

    fd = None
    old_terminal = None

    try:
        power = ask_power()

        print()
        print(f"Using motors: {LEFT_MOTOR}=power, {RIGHT_MOTOR}=-power")
        print(f"Camera: {CAMERA}")
        print()
        print("Controls while running:")
        print("  s = stop motors but keep printing markers")
        print("  r = resume rotation")
        print("  q = stop motors and quit")
        print()

        rotating = True

        fd = sys.stdin.fileno()
        old_terminal = termios.tcgetattr(fd)
        tty.setcbreak(fd)

        while True:
            if rotating:
                set_rotation(io, power)
            else:
                stop_drive(io)

            try:
                markers = io.camera[CAMERA].see()
                print_markers(markers)
            except Exception as exc:
                print(f"camera error: {exc}")

            if key_pressed():
                key = sys.stdin.read(1).lower()

                if key == "s":
                    rotating = False
                    stop_drive(io)
                    print("rotation stopped; camera scan continues")

                elif key == "r":
                    rotating = True
                    print("rotation resumed")

                elif key == "q":
                    print("quitting")
                    break

            time.sleep(SCAN_INTERVAL_S)

    finally:
        try:
            stop_drive(io)
        except Exception:
            pass

        if fd is not None and old_terminal is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_terminal)
            except Exception:
                pass

        io.close()


if __name__ == "__main__":
    main()