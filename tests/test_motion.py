from tests.registry import register_test


@register_test(category="motion", enabled=False)
def test_level2_drive(robot):
    """Visible movement test in simulator"""
    from level2_canonical import Level2

    print("Testing Level 2 DRIVE...")
    lvl2 = Level2(robot)

    lvl2.DRIVE(0.5, 0.5, duration=1.0)
    lvl2.DRIVE(-0.5, -0.5, duration=1.0)
    lvl2.ROTATE(90)
    lvl2.ROTATE(-90)
