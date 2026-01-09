# markers.py

ARENA_MARKERS = range(0, 20)
ACIDIC_MARKERS = range(100, 140)
BASIC_MARKERS = range(140, 180)

def marker_type(marker_id):
    if marker_id in ARENA_MARKERS:
        return "arena"
    if marker_id in ACIDIC_MARKERS:
        return "acidic"
    if marker_id in BASIC_MARKERS:
        return "basic"
    return "unknown"
