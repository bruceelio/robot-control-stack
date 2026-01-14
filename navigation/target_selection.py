# navigation/target_selection.py

"""
Target selection utilities.

Pure logic. No motion, no hardware access.
Safe for use in behaviors, planning, and tests.
"""


def get_closest_target(perception, kind):
    """
    Return the closest currently-known target of the given kind.
    Uses perception object memory.
    """
    memory = perception.objects.get(kind)

    if not memory:
        return None

    # memory is a dict: id -> target
    return min(memory.values(), key=lambda t: t["distance"])