from tests.registry import register_test


@register_test(category="motion", enabled=True)
def test_level2_drive(robot):
    """Visible movement test in simulator"""
    from level2_canonical import Level2

    print("Testing Level 2 DRIVE...")
    lvl2 = Level2(robot)

    lvl2.DRIVE(0.5, 0.5, duration=1.0)
    lvl2.DRIVE(-0.5, -0.5, duration=0.5)
    lvl2.ROTATE(90)
    lvl2.ROTATE(-90)

@register_test(category="motion", enabled=False)
def test_level2_lift(robot):
    """Visible lift up/down test in simulator"""
    from level2_canonical import Level2
    from primitives.manipulation import LiftUp, LiftDown
    from primitives.base import PrimitiveStatus
    import time

    print("Testing Lift primitives...")
    lvl2 = Level2(robot)

    lift_up = LiftUp(settle_time=0.5)
    lift_up.start(lvl2=lvl2)

    while lift_up.update() == PrimitiveStatus.RUNNING:
        time.sleep(0.05)

    lift_down = LiftDown(settle_time=0.5)
    lift_down.start(lvl2=lvl2)

    while lift_down.update() == PrimitiveStatus.RUNNING:
        time.sleep(0.05)

    print("Lift test complete")
