# Versioning & Release Policy

This project uses a **semantic-style versioning scheme** tailored for a
robot control stack that runs in both simulation and real hardware.

## Version Format

MAJOR.MINOR.PATCH


## Meaning

### MAJOR
Architectural or behavioral contract changes.

Examples:
- Configuration schema redesign
- Control flow or behavior lifecycle changes
- Breaking API changes

### MINOR
New capabilities added.

Examples:
- New behaviors
- New robot profiles
- Navigation or perception improvements

### PATCH
Fixes and tuning.

Examples:
- Calibration adjustments
- Bug fixes
- Safety improvements

## Version Lifecycle

- Commits are made frequently
- Versions are **tagged only at known-good milestones**
- Tags are immutable once created
- Versions below `1.0.0` indicate an evolving API

## Examples

- `v0.1.0` – First successful simulation boot
- `v0.2.0` – Configuration system consolidated
- `v0.2.1` – Perception edge-case fix
- `v0.3.0` – Multi-robot profile support
- `v1.0.0` – First competition-ready release

## Philosophy

- Commit often
- Tag rarely
- Version conservatively

If unsure whether something deserves a new version, it probably doesn’t yet.

