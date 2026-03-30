# config/arena_tags.py

from config import arena

_TAG_SIZE_MAP: dict[int, float] | None = None


def build_tag_size_map() -> dict[int, float]:
    size_map: dict[int, float] = {}

    for group_name, tag_ids in arena.APRILTAG_ID_GROUPS.items():
        if group_name not in arena.APRILTAG_SIZE_BY_GROUP_M:
            raise RuntimeError(
                f"APRILTAG_ID_GROUPS contains group {group_name!r} "
                f"but APRILTAG_SIZE_BY_GROUP_M has no size for it"
            )

        size_m = arena.APRILTAG_SIZE_BY_GROUP_M[group_name]

        for tag_id in tag_ids:
            if tag_id in size_map:
                raise RuntimeError(
                    f"AprilTag id {tag_id} appears in more than one configured group"
                )
            size_map[tag_id] = size_m

    return size_map


def get_tag_size_map() -> dict[int, float]:
    global _TAG_SIZE_MAP
    if _TAG_SIZE_MAP is None:
        _TAG_SIZE_MAP = build_tag_size_map()
    return _TAG_SIZE_MAP


def resolve_tag_size_m(tag_id: int) -> float:
    try:
        return get_tag_size_map()[tag_id]
    except KeyError as e:
        raise RuntimeError(
            f"No configured AprilTag size for tag id {tag_id}. "
            f"Check APRILTAG_ID_GROUPS / APRILTAG_SIZE_BY_GROUP_M."
        ) from e