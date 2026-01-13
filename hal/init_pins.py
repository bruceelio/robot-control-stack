# init_pins.py
"""
Universal I/O setup.
- Sets pin modes for any Arduino/board using canonical mapping.
- Silent if SR3 API is being used (pins already handled).
"""

try:
    from sr.robot3 import Robot, INPUT, INPUT_PULLUP, OUTPUT
    SR3_PRESENT = True
except ImportError:
    SR3_PRESENT = False

from .pinmap import board_mapping

# Track initialized pins to avoid duplicates
_initialized_pins = set()

def setup_boards():
    if SR3_PRESENT:
        # SR API handles pins; skip setup
        return

    # If SR3 is not present, manually set pin modes
    for board_name, mapping in board_mapping.items():
        for canonical_name, pin in mapping.items():
            if pin is None or (board_name, pin) in _initialized_pins:
                continue

            # Choose mode based on canonical type
            if canonical_name.startswith("DI_"):
                mode = "INPUT_PULLUP"  # default for microswitches
            elif canonical_name.startswith("DO_"):
                mode = "OUTPUT"
            else:
                continue  # AI/AO handled elsewhere

            # Example: replace with actual hardware library call
            # e.g., ArduinoPy or RPi.GPIO
            print(f"[io_setup] Setting {board_name} pin {pin} ({canonical_name}) mode={mode}")

            # Mark initialized
            _initialized_pins.add((board_name, pin))

def initialize():
    """Call this at program start to setup all pins."""
    setup_boards()


