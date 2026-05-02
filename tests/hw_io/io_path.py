"""Small parser for the agreed Pi-facing IO path convention.

Supported examples:
    io.bumper["front_left"]
    io.motor["shooter"].power
    io.imu.heading
    io.camera["front"].see()
"""

from dataclasses import dataclass
import re
from typing import Optional

_PATH_RE = re.compile(
    r'^io\.(?P<category>[A-Za-z_][A-Za-z0-9_]*)'
    r'(?:\["(?P<name>[^"]+)"\])?'
    r'(?:\.(?P<member>[A-Za-z_][A-Za-z0-9_]*)(?P<call>\(\))?)?$'
)


@dataclass(frozen=True)
class ParsedIOPath:
    raw: str
    category: str
    name: Optional[str] = None
    member: Optional[str] = None
    is_call: bool = False

    @property
    def rule_prefix(self) -> str:
        return f"io.{self.category}"


def parse_io_path(path: str) -> ParsedIOPath:
    text = path.strip()
    match = _PATH_RE.match(text)
    if not match:
        raise ValueError(f"Invalid IO path format: {path}")
    return ParsedIOPath(
        raw=text,
        category=match.group("category"),
        name=match.group("name"),
        member=match.group("member"),
        is_call=bool(match.group("call")),
    )


def resolve_target(io, parsed: ParsedIOPath):
    target = getattr(io, parsed.category)
    if parsed.name is not None:
        target = target[parsed.name]
    return target


def read_io_path(io, path: str):
    parsed = parse_io_path(path)
    target = resolve_target(io, parsed)
    if parsed.member is None:
        return target
    member = getattr(target, parsed.member)
    return member() if parsed.is_call else member


def write_io_path(io, path: str, value):
    parsed = parse_io_path(path)
    if parsed.member is None or parsed.is_call:
        raise ValueError(f"IO path is not writable: {path}")
    target = resolve_target(io, parsed)
    setattr(target, parsed.member, value)
