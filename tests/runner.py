from tests.registry import TESTS


def run_tests(
    robot=None,
    *,
    only=None,
    category=None,
):
    """
    Run registered tests.

    only="test_name"     -> run a single test
    category="hal"       -> run tests in category
    """
    print("\n=== TEST RUN START ===")

    for name, meta in TESTS.items():
        if not meta["enabled"]:
            continue
        if only and name != only:
            continue
        if category and meta["category"] != category:
            continue

        print(f"\n>>> RUNNING TEST: {name}")
        try:
            if meta["requires_robot"]:
                if robot is None:
                    raise RuntimeError("Robot instance required")
                meta["func"](robot)
            else:
                meta["func"]()
        except Exception as e:
            print(f"!!! TEST FAILED: {name}")
            print(f"    {e}")

    print("\n=== TEST RUN COMPLETE ===\n")
