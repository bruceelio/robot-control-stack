zone_0/
  robot.py
  robot_controller.py
  state_machine.py
  README.md

  behaviors/
    base.py
    init_escape.py
    seek_and_collect.py
    post_pickup_realign.py
    post_dropoff_realign.py
    recover_localisation.py
    return_to_base.py

  primitives/
    base.py
    motion.py
    manipulation.py
    sensing.py
    system.py

  composites/
    drive_then_rotate.py

  motion_backends/
    base.py
    timed.py
    encoder.py

  level2/
    level2_canonical.py

  hw_io/
    base.py
    resolve.py
    sr1.py
    sr_board.py
    cameras/
      base.py
      sr_april.py

  navigation/
    localisation.py
    geometry.py
    height_model.py
    markers.py
    navigator.py
    rotate_and_drive.py
    target_selection.py
    legacy.py

  config/
    arena.py
    schema.py
    strategy.py
    profiles/
      simulation.py
      sr1.py

  calibration/
    resolve.py
    schema.py
    profiles/
      simulation.py
      sr1.py

  diagnostics/
  tests/
  docs/
  legacy/                 # keep your HAL here as you already do
    hal/
      hardware.py
      pinmap.py
      init_pins.py
      aux_board.py
