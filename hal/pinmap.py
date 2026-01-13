# pinmap.py
"""
Unified canonical-to-pin mapping for all robot boards.
Handles multiple boards and missing extra boards gracefully.
"""

from .sr_board import sr_robot_io, SR_ROBOT_BOARD_NAME

# Initialize unified mapping with SR Robot board
canonical_to_pin = {}
board_mapping = {SR_ROBOT_BOARD_NAME: sr_robot_io}

# Attempt to load extra boards if they exist
try:
    from .aux_board import extra_board_io, EXTRA_BOARD_NAME
    board_mapping[EXTRA_BOARD_NAME] = extra_board_io
except ImportError:
    print("No extra board found. Continuing with SR Robot only.")

# Merge all board mappings into a single canonical_to_pin dictionary
for board_name, mapping in board_mapping.items():
    for canonical_name, pin in mapping.items():
        if pin is None:
            continue  # Skip unmapped pins
        if canonical_name in canonical_to_pin:
            print(f"Warning: {canonical_name} already mapped "
                  f"({canonical_to_pin[canonical_name]}) – overriding with {pin} from {board_name}")
        canonical_to_pin[canonical_name] = pin

# Optional: print summary for debug
if __name__ == "__main__":
    print("\n=== Unified Canonical-to-Pin Mapping ===")
    for name, pin in canonical_to_pin.items():
        print(f"{name}: {pin}")
    print("========================================\n")
