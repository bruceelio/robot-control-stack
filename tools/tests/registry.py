# ------------------------------------------------------------------
# Test registry
# ------------------------------------------------------------------

TESTS = {}


def register_test(
    *,
    name=None,
    category="general",
    enabled=True,
    requires_robot=True,
):
    """
    Decorator to register a test.

    enabled=True   -> test is runnable
    enabled=False  -> test is skipped
    requires_robot -> False for pure logic / virtual tests
    """
    def decorator(fn):
        test_name = name or fn.__name__
        TESTS[test_name] = {
            "func": fn,
            "category": category,
            "enabled": enabled,
            "requires_robot": requires_robot,
        }
        return fn
    return decorator
