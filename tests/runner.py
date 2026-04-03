# tests/runner.py
from tests.registry import TESTS


def run_tests(
    robot=None,
    *,
    only="test_mega_serial_drive_and_grip",
    category=None,
):
    """
    Run registered tests.

    only="test_name"  -> run a single test (not file but test name)
    category="io"     -> run tests in that category
    """
    print("\n=== TEST RUN START ===")

    # Select tests
    selected = []
    for test_name, meta in TESTS.items():
        if only and test_name != only:
            continue
        if category and meta.get("category") != category:
            continue
        if not meta.get("enabled", True):
            continue
        if meta.get("requires_robot", True) and robot is None:
            continue
        selected.append((test_name, meta))

    if not selected:
        print("No tests selected.")
        print(f"Requested: only={only} category={category}")
        print("Available tests:")
        for test_name, meta in TESTS.items():
            print(
                f"  - {test_name} (category={meta.get('category')}, enabled={meta.get('enabled')})"
            )
        print("=== TEST RUN END ===\n")
        return False

    ok = True
    for test_name, meta in selected:
        fn = meta["func"]
        print(f"\n--- RUN {test_name} (category={meta.get('category')}) ---")
        try:
            if meta.get("requires_robot", True):
                fn(robot)
            else:
                fn()
            print(f"[PASS] {test_name}")
        except Exception as e:
            ok = False
            print(f"[FAIL] {test_name}: {e}")

    print("\n=== TEST RUN END ===\n")
    return ok
