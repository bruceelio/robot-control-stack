navigation/
├── __init__.py
├── navigator.py
├── README.md
├── README_NAVIGATION_FILE_HIERARCHY.md
│
├── command/
│   ├── __init__.py
│   ├── models.py
│   ├── motor_output_conditioner.py
│   └── velocity_arbiter.py
│
├── control/
│   ├── __init__.py
│   ├── dual_range_controller.py
│   ├── models.py
│   ├── range_only_controller.py
│   ├── distance_angle_controller.py
│   ├── smooth_control_law.py
│   └── smooth_fov_admissibility.py
│
├── geometry/
│   ├── __init__.py
│   ├── angles.py
│   ├── line_projection.py
│   ├── models.py
│   ├── ray_line_intersection.py
│   ├── relative_goal_pose.py
│   ├── relative_target.py
│   ├── transforms_2d.py
│   └── transforms_3d.py
│
├── guidance/
│   ├── __init__.py
│   ├── target_curvature.py
│   ├── bearing_only_guidance.py
│   ├── go_to_goal.py
│   ├── line_of_sight.py
│   ├── models.py
│   ├── vector_field_guidance.py
│   └── waypoint_guidance.py
│
├── kinematics/
│   ├── __init__.py
│   ├── differential_drive.py
│   ├── models.py
│   ├── pose_integration.py
│   └── unicycle.py
│
├── line_following/
│   ├── __init__.py
│   ├── line_follower.py
│   └── models.py
│
├── local_planning/
│   ├── __init__.py
│   ├── artificial_potential_field.py
│   ├── bug2.py
│   ├── dynamic_window.py
│   ├── follow_the_gap.py
│   ├── local_planning_coordinator.py
│   ├── models.py
│   ├── README_NAV_LOCAL_PLANNING.md
│   ├── tangent_bug.py
│   ├── vector_field_histogram.py
│   └── velocity_obstacle.py
│
├── motion_planning/
│   ├── __init__.py
│   ├── dubins.py
│   ├── models.py
│   ├── motion_primitives.py
│   ├── README_NAV_MOTION_PLANNING.md
│   ├── reeds_shepp.py
│   ├── state_lattice.py
│   └── trajectory/
│       ├── __init__.py
│       ├── cubic_polynomial.py
│       ├── models.py
│       ├── quintic_polynomial.py
│       ├── time_parameterization.py
│       └── trapezoidal_profile.py
│
├── odometry/
│   ├── __init__.py
│   ├── base.py
│   ├── drive_encoders.py
│   ├── README_NAVIGATION_ODOMETRY.md
│   ├── resolve.py
│   └── three_deadwheel.py
│
├── path_planning/
│   ├── __init__.py
│   ├── models.py
│   ├── README_NAV_PATH_PLANNING.md
│   │
│   ├── geometric/
│   │   ├── __init__.py
│   │   └── visibility_graph.py
│   │
│   ├── grid/
│   │   ├── __init__.py
│   │   ├── a_star.py
│   │   ├── dijkstra.py
│   │   └── theta_star.py
│   │
│   ├── sampling/
│   │   ├── __init__.py
│   │   ├── rrt.py
│   │   └── rrt_star.py
│   │
│   └── smoothing/
│       ├── __init__.py
│       ├── cubic_spline.py
│       ├── polynomial_spline.py
│       └── shortcut_smoothing.py
│
├── path_tracking/
│   ├── __init__.py
│   ├── cross_track_controller.py
│   ├── models.py
│   ├── path_tracking_arbiter_mecanum.py
│   ├── path_tracking_arbiter_nonmec.py
│   ├── README_NAV_PATH_TRACKING.md
│   │
│   └── pure_pursuit/
│       ├── __init__.py
│       ├── pure_pursuit_mecanum.py
│       ├── pure_pursuit_nonmec.py
│       └── README_PURE_PURSUIT.md
│
├── trajectory_tracking/
│   ├── __init__.py
│   ├── models.py
│   └── ramsete.py
│
├── visual_servoing/
│   ├── __init__.py
│   ├── ibvs_bearing_only_tracking.py
│   ├── ibvs_target_centering.py
│   ├── interaction_matrix.py
│   ├── models.py
│   ├── pbvs_fov_visibility_constraint.py
│   ├── pbvs_holonomic.py
│   ├── pbvs_nonholonomic.py
│   ├── pose_servo_controller.py
│   └── servoing_types.py
│
├── wall_following/
│   ├── __init__.py
│   ├── distance_only.py
│   ├── heading_distance.py
│   ├── heading_only.py
│   ├── models.py
│   ├── README_NAV_WALL_FOLLOWING.md
│   └── wall_follower.py
│
└── wall_geometry/
    ├── __init__.py
    ├── models.py
    ├── resolver.py
    ├── tof_pair.py
    ├── tof_scan.py
    ├── two_ray_plane.py
    ├── ultrasonic_pair.py
    └── ultrasonic_scan.py