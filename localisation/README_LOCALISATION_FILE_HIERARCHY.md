localisation/
│
├── __init__.py
├── localisation.py
├── models.py
├── README_ARBITRATION.md
├── README_LOCALISATION.md
│
├── arbitration/
│   ├── __init__.py
│   ├── pose_arbiter.py
│   └── scoring.py
│
├── estimation/
│   ├── __init__.py
│   │
│   └── vision/
│       ├── __init__.py
│       ├── apriltag_pnp.py
│       ├── cam1_markers1.py
│       ├── cam1_markers2.py
│       ├── cam1_markers3.py
│       ├── cam2_markers2.py
│       └── vision_arbiter.py
│
├── fusion/
│   ├── __init__.py
│   └── models.py
│
├── geometry/
│   ├── __init__.py
│   ├── bearing_resection_2d.py
│   ├── circle_intersection.py
│   ├── range_multilateration_2d.py
│   └── range_to_lines_2d.py
│
└── propagation/
    ├── __init__.py
    ├── commanded_motion.py
    └── odometry.py