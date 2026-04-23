import matplotlib.pyplot as plt
import math

arena_size = 4575
half = arena_size / 2

poses = [
    (-1875.5, 1875.5, -45),
    (-1652.0, 1652.2, -44.9772249084063),
    (-1414.0, 1414.0, -44.9772249084063),
    (-1372.0, 1372.2, -44.9772249084063),
    (-1368.0, 1368.7, -44.9772249084063),
    (-808.3, 808.3, -44.9772249084063),
    (-808.3, 808.3, 179.994843375488),
    (-808.3, 808.3, 164.994795628965),
    (-1532.0, 1002.4, 164.994795628965),
    (-1291.0, 937.7, 164.994795628965),
    (-1291.0, 937.7, 74.9945091498254),
    (-1291.0, 937.7, 59.9944614033021),
    (-1291.0, 937.7, 44.9944136567789),
    (-1291.0, 937.7, 29.9943659102556),
    (-1291.0, 937.7, 14.9943181637324),
    (-1291.0, 937.7, -105.195140040553),
    (-1291.0, 937.7, -14.9542110841962),
    (-1291.0, 937.7, -29.9657179963012),
    (-1291.0, 937.7, -44.9772249084063),
    (-1291.0, 937.7, -64.4005105694887),
    (-1157.0, 658.6, -64.4005105694887),
    (-1157.0, 658.6, -94.4235243936987),
    (-1157.0, 658.6, -79.4120174815937),
    (-1157.0, 658.6, -59.7595485088761),
    (-971.8, 338.9, -59.7595485088761),
    (-940.0, 284.1, -59.7595485088761),
    (-937.4, 279.8, -59.7595485088761),
    (-539.1, -404.7, -59.7595485088761),
    (-539.1, -404.7, 165.195331026646),
    (-539.1, -404.7, 150.195283280122),
    (-1189.0, -31.9, 150.195283280122),
    (-973.0, -156.2, 150.195283280122),
    (-1381.0, 672.3, 126.428973863553),
    (-685.6, 68.9, 179.994843375488),
    (-435.6, 68.9, 179.994843375488),
    (-435.6, 68.9, 89.9945568963487),
    (-435.6, 68.9, 74.9945091498254),
    (-908.2, 554.7, 6.02752109600553),
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