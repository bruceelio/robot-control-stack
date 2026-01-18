from tests.registry import register_test


@register_test(category="safety", enabled=True, requires_robot=False)
def test_virtual_front_bumper():
    """Tests SI_BUMPER_FRONT logic"""
    from legacy.hal import (
        DI_BUMPER_FRONT_LEFT,
        DI_BUMPER_FRONT_RIGHT,
        SI_BUMPER_FRONT,
        update_virtual_bumpers,
    )

    print("Testing SI_BUMPER_FRONT...")

    def mock_read(di):
        return di in (DI_BUMPER_FRONT_LEFT, DI_BUMPER_FRONT_RIGHT)

    bumpers = update_virtual_bumpers(mock_read)
    print("SI_BUMPER_FRONT =", bumpers[SI_BUMPER_FRONT])
