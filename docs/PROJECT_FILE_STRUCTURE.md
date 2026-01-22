Directory Tree
 
│   perception.py
│   README.md
│   robot.py
│   robot_controller.py
│   state_machine.py
│
├───.idea
│   │   .gitignore
│   │   .name
│   │   misc.xml
│   │   modules.xml
│   │   vcs.xml
│   │   workspace.xml
│   │   zone_0.iml
│   │
│   └───inspectionProfiles
│           profiles_settings.xml
│
├───behaviors
│   │   base.py
│   │   init_escape.py
│   │   post_dropoff_realign.py
│   │   post_pickup_realign.py
│   │   recover_localisation.py
│   │   return_to_base.py
│   │   seek_and_collect.py
│   │   __init__.py
│   │
│   └───__pycache__
│           base.cpython-313.pyc
│           init_escape.cpython-313.pyc
│           post_dropoff_realign.cpython-313.pyc
│           post_pickup_realign.cpython-313.pyc
│           recover_localisation.cpython-313.pyc
│           return_to_base.cpython-313.pyc
│           seek_and_collect.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───calibration
│   │   base_legacy.py
│   │   inspect.py
│   │   README.md
│   │   resolve.py
│   │   schema.py
│   │   __init__.py
│   │
│   ├───profiles
│   │   │   simulation.py
│   │   │   sr1.py
│   │   │
│   │   └───__pycache__
│   │           simulation.cpython-313.pyc
│   │
│   └───__pycache__
│           resolve.cpython-313.pyc
│           schema.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───canonical
│       canonical.py
│
├───config
│   │   arena.py
│   │   README.md
│   │   schema.py
│   │   strategy.py
│   │   __init__.py
│   │
│   ├───profiles
│   │   │   simulation.py
│   │   │   sr1.py
│   │   │   __init__.py
│   │   │
│   │   └───__pycache__
│   │           simulation.cpython-313.pyc
│   │           sr1.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │
│   └───__pycache__
│           arena.cpython-313.pyc
│           schema.cpython-313.pyc
│           strategy.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───diagnostics
│   │   camera_angles.py
│   │   drive_timing.py
│   │   README.md
│   │   registry.py
│   │   report.py
│   │   rotation_calibration.py
│   │   rotation_timing.py
│   │   runner.py
│   │   __init__.py
│   │
│   └───__pycache__
│           camera_angles.cpython-313.pyc
│           drive_timing.cpython-313.pyc
│           rotation_calibration.cpython-313.pyc
│           runner.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───docs
│   │   ARCHITECTURE.md
│   │   PROJECT_FILE_STRUCTURE.md
│   │   RUNTIME_FLOW.md
│   │   Sample Logic Flowchart.jpg
│   │   TESTING_AND_DIAGNOSTICS.md
│   │   VERSIONING.md
│   │
│   └───competitions
│       └───SR_COMP_2026
│               SR_COMP_2026_ ARENA.svg
│               SR_COMP_2026_PROGRAMMING.md
│               SR_COMP_2026_RULES.md
│               SR_COMP_2026_SIMULATOR
│
├───hw_io
│   │   base.py
│   │   README.md
│   │   resolve.py
│   │   sr1.py
│   │   sr_board.py
│   │   __init__.py
│   │
│   ├───cameras
│   │   │   base.py
│   │   │   sr_april.py
│   │   │   __init__.py
│   │   │
│   │   └───__pycache__
│   │           base.cpython-313.pyc
│   │           sr_april.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │
│   └───__pycache__
│           base.cpython-313.pyc
│           resolve.cpython-313.pyc
│           sr1.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───legacy
│   │   motion.py
│   │   nonproject_tests.py
│   │   robots.py
│   │
│   └───hal
│           aux_board.py
│           hardware.py
│           init_pins.py
│           pinmap.py
│           README.md
│           __init__.py
│
├───level2
│   │   level2_canonical.py
│   │
│   └───__pycache__
│           level2_canonical.cpython-313.pyc
│
├───localisation
│   │   arbitration.py
│   │   localisation.py
│   │   localisation_temp.py
│   │   pose_types.py
│   │   README.md
│   │   __init__.py
│   │
│   ├───providers
│   │   │   base.py
│   │   │   cam1.markers1.py
│   │   │   cam1_markers2.py
│   │   │   cam1_markers3.py
│   │   │   cam2_markers2.py
│   │   │   __init__.py
│   │   │
│   │   └───__pycache__
│   │           base.cpython-313.pyc
│   │           cam1_markers2.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │
│   └───__pycache__
│           localisation.cpython-313.pyc
│           pose_types.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───motion_backends
│   │   base.py
│   │   encoder.py
│   │   timed.py
│   │   __init__.py
│   │
│   └───__pycache__
│           base.cpython-313.pyc
│           timed.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───navigation
│   │   geometry.py
│   │   height_model.py
│   │   legacy.py
│   │   markers.py
│   │   navigator.py
│   │   README.md
│   │   __init__.py
│   │
│   └───__pycache__
│           geometry.cpython-313.pyc
│           height_model.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───primitives
│   │   base.py
│   │   manipulation_legacy.py
│   │   motion_legacy.py
│   │   sensing_legacy.py
│   │   system_legacy.py
│   │   __init__.py
│   │
│   ├───manipulation
│   │   │   grab.py
│   │   │   liftdown.py
│   │   │   liftup.py
│   │   │   release.py
│   │   │   __init__.py
│   │   │
│   │   └───__pycache__
│   │           grab.cpython-313.pyc
│   │           liftdown.cpython-313.pyc
│   │           liftup.cpython-313.pyc
│   │           release.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │
│   ├───motion
│   │   │   drive.py
│   │   │   rotate.py
│   │   │   stop.py
│   │   │   __init__.py
│   │   │
│   │   └───__pycache__
│   │           drive.cpython-313.pyc
│   │           rotate.cpython-313.pyc
│   │           stop.cpython-313.pyc
│   │           __init__.cpython-313.pyc
│   │
│   ├───sensing
│   │       __init__.py
│   │
│   ├───system
│   │       __init__.py
│   │
│   └───__pycache__
│           base.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───skills
│   │   drive_then_rotate.py
│   │   rotate_then_drive.py
│   │   SelectTarget.py
│   │   __init__.py
│   │
│   └───__pycache__
│           drive_then_rotate.cpython-313.pyc
│           SelectTarget.cpython-313.pyc
│           __init__.cpython-313.pyc
│
├───tests
│   │   README.md
│   │   registry.py
│   │   runner.py
│   │   test_hal_io.py
│   │   test_io_checkout.py
│   │   test_motion.py
│   │   test_safety.py
│   │   __init__.py
│   │
│   └───__pycache__
│           registry.cpython-313.pyc
│           runner.cpython-313.pyc
│           test_io_checkout.cpython-313.pyc
│           test_motion.cpython-313.pyc
│           __init__.cpython-313.pyc
│
└───__pycache__
        arena_marker_coordinates.cpython-313.pyc
        perception.cpython-313.pyc
        robot_controller.cpython-313.pyc
        state_machine.cpython-313.pyc