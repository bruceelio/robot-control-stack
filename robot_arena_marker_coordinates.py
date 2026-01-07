from arena_marker_coordinates import MARKER_POSITIONS

# Example usage inside your robot code
markers_seen = robot.camera.find_all_markers()

for m in markers_seen:
    world_x, world_y = MARKER_POSITIONS[m.id]
    print(
        f"Marker {m.id}: world=({world_x:.2f}, {world_y:.2f}) "
        f"distance={m.position.distance:.0f}mm "
        f"angle={m.position.horizontal_angle:.2f}rad"
    )
