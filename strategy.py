# strategy.py
from config import DEFAULT_COLLECT_MODE
from perception import get_visible_markers
import math
import logging

# -----------------------------
# Setup Logging
# -----------------------------
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

# -----------------------------
# Strategy Configuration
# -----------------------------
COLLECT_MODE = DEFAULT_COLLECT_MODE  # "acidic" or "basic"
STEP_DISTANCE = 200                  # mm per navigation step
STOP_DISTANCE = 100                  # mm to consider target reached

# -----------------------------
# Helper Functions
# -----------------------------
def distance(point1, point2):
    """Euclidean distance between two points (x, y)."""
    dx = point1[0] - point2[0]
    dy = point1[1] - point2[1]
    return math.hypot(dx, dy)


def select_target(robot_pos, visible_markers, collect_mode):
    """
    Decide which marker to go for next.
    Strategy:
        1. Prefer visible markers of COLLECT_MODE type.
        2. If none visible, choose the closest known starting position of that type.
    """
    # Filter visible markers by type
    visible_of_type = [m for m in visible_markers if m['type'] == collect_mode]

    if visible_of_type:
        # Choose the closest visible marker
        target = min(visible_of_type, key=lambda m: distance(robot_pos, m['position']))
        logging.info(f"Targeting visible {collect_mode} marker {target['id']} at {target['position']}")
        return target['position']

    # Fallback: no visible marker, go to closest starting position
    known_positions = [m['start_pos'] for m in visible_markers if m['type'] == collect_mode]
    if known_positions:
        target_pos = min(known_positions, key=lambda pos: distance(robot_pos, pos))
        logging.info(f"No visible {collect_mode} marker; heading toward known start {target_pos}")
        return target_pos

    # Nothing to go for
    logging.warning(f"No {collect_mode} markers known or visible")
    return None


# -----------------------------
# Main Strategy Loop
# -----------------------------
def run_strategy(robot, current_pos):
    """
    Main strategy loop.
    current_pos: robot's initial position (x, y)
    """
    global COLLECT_MODE

    while True:
        # Step 1: Get marker perceptions
        visible_markers = get_visible_markers(robot)
        logging.info(f"{len(visible_markers)} markers visible")

        # Step 2: Decide target
        target_pos = select_target(current_pos, visible_markers, COLLECT_MODE)
        if target_pos is None:
            logging.info("No targets remaining, strategy complete")
            break

        # Step 3: Rotate toward target
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        angle = math.degrees(math.atan2(dy, dx))
        logging.info(f"Rotating {angle:.1f}° toward target")
        from navigation import rotate_degrees
        rotate_degrees(robot, angle)

        # Step 4: Drive a step
        distance_to_target = distance(current_pos, target_pos)
        step = min(STEP_DISTANCE, distance_to_target)
        logging.info(f"Driving {step:.1f} mm toward target")
        from navigation import drive_step
        drive_step(robot, step)

        # Step 5: Update position (simulated)
        current_pos = (current_pos[0] + step * math.cos(math.radians(angle)),
                       current_pos[1] + step * math.sin(math.radians(angle)))

        # Step 6: Check if target reached
        if distance(current_pos, target_pos) < STOP_DISTANCE:
            logging.info(f"Reached target at {target_pos}")

# -----------------------------
# Example run
# -----------------------------
if __name__ == "__main__":
    # Placeholder starting position
    initial_pos = (0, 0)
    run_strategy(robot=None, current_pos=initial_pos)
