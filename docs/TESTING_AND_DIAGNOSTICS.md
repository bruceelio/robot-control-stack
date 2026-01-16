# Testing & Diagnostics

Testing and diagnostics are **explicit modes**, never accidental.

They exist to catch failures *before* the robot does.

---

## Testing Principles

- Tests should fail before the robot does
- Logic must be testable without real hardware
- Every serious bug earns a regression test
- Tests must not depend on global runtime state

---

## Diagnostics

Diagnostics are allowed to:
- Bypass normal behavior sequencing
- Execute focused checks and calibration routines

Diagnostics must NOT:
- Bypass configuration
- Bypass safety limits
- Permanently mutate runtime state

Diagnostics exist to **observe and validate**, not to “cheat” execution.