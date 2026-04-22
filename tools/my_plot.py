import matplotlib.pyplot as plt
import math

arena_size = 4575
half = arena_size / 2

poses = [
    (-1875.5, 1875.5, -45),
    (-1638.0, 1638.9, -44.9772249084063),
    (-1402.0, 1402.6, -44.9772249084063),
    (-1388.0, 1388.7, -44.9772249084063),
    (-829.7, 829.7, -44.9772249084063),
    (-829.7, 829.7, 179.994843375488),
    (-829.7, 829.7, 164.994795628965),
    (-1554.0, 1023.9, 164.994795628965),
    (-1312.0, 959.1, 164.994795628965),
    (-1312.0, 959.1, 74.9945091498254),
    (-1312.0, 959.1, 59.9944614033021),
    (-1312.0, 959.1, 44.9944136567789),
    (-1312.0, 959.1, 29.9943659102556),
    (-1312.0, 959.1, 14.9943181637324),
    (-1312.0, 959.1, -105.195140040553),
    (-1312.0, 959.1, -14.9542110841962),
    (-1312.0, 959.1, -31.7991844893828),
    (-949.7, 733.7, -31.7991844893828),
    (-581.0, 504.8, -31.7991844893828),
    (-560.5, 492.0, -31.7991844893828),
    (112.0, 74.4, -31.7991844893828),
    (112.0, 74.4, -166.78815504251),
    (112.0, 74.4, 178.155647299616),
    (-637.6, 98.5, 178.155647299616),
    (-387.7, 90.5, 178.155647299616),
    (-387.7, 90.5, 88.1553608204762),
    (126.3, 1396.0, -147.192981897701),
    (336.5, 1531.4, -147.192981897701),
    (336.5, 1531.4, 122.796418374135),
]

# Optional: your observed actual end point
actual_end = (-1732, 724)

xs = [p[0] for p in poses]
ys = [p[1] for p in poses]

fig, ax = plt.subplots(figsize=(8, 8))

# Arena boundary
ax.plot(
    [-half, half, half, -half, -half],
    [-half, -half, half, half, -half]
)

# Path
ax.plot(xs, ys, marker='o')

# Start and end labels
ax.text(xs[0], ys[0], "start")
ax.text(xs[-1], ys[-1], "logged end")
ax.plot(actual_end[0], actual_end[1], marker='x', markersize=10)
ax.text(actual_end[0], actual_end[1], "actual end")

# Heading arrows every few points
arrow_len = 120
for i, (x, y, hdg_deg) in enumerate(poses):
    if i % 3 == 0:
        hdg = math.radians(hdg_deg)
        dx = arrow_len * math.cos(hdg)
        dy = arrow_len * math.sin(hdg)
        ax.arrow(x, y, dx, dy, head_width=40, length_includes_head=True)

ax.set_xlim(-half - 100, half + 100)
ax.set_ylim(-half - 100, half + 100)
ax.set_aspect('equal')
ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm)")
ax.set_title("Robot path in arena")
# Define 6 equal divisions across the arena
step = arena_size / 6  # 4575 / 6 = 762.5

# Create tick positions (7 ticks = 6 divisions)
ticks = [(-half + i * step) for i in range(7)]

# Apply ticks to both axes
ax.set_xticks(ticks)
ax.set_yticks(ticks)

# Draw grid aligned with these divisions
ax.grid(True)

plt.show()