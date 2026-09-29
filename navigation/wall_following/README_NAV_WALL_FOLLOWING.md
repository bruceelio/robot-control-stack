# Wall Following

This package contains reusable, hardware-independent wall-following
algorithms.

It does not read sensors, access robot IO, or control motors directly.

## Inputs

Wall-following algorithms operate on semantic navigation information,
primarily:

- wall distance;
- wall heading;
- desired wall distance;
- desired forward velocity.

Wall geometry is represented by:

```python
navigation.wall_geometry.WallGeometry

A wall geometry estimate may contain:
- heading only;
- distance only;
- heading and distance.
Intended controller variants
The package will initially support:
- distance-only wall following;
- heading-only wall-relative control;
- combined heading + distance wall following;
- a master selector which chooses the strongest available method.
The controller output follows the canonical navigation velocity
convention:
- cmd_vel_linear_x_mps
- cmd_vel_lateral_y_mps
- cmd_vel_angular_z_rps
Separation of concerns
navigation/wall_following/
contains control algorithms.
navigation/providers/
contains algorithms which derive wall geometry from observations.
skills/navigation/
owns robot-facing procedures such as acquiring measurements, selecting
a wall, applying commands, handling loss of observations, and deciding
when an operation has completed.