# Robotics Log State-Change Summarization Instructions

## Goal
Given a robotics log file, return a filtered `.txt` log that:
- Removes repeated information
- Preserves **state changes**
- Ignores cosmetic differences such as timestamps
- Keeps original line formatting for all retained lines

The output must be a plain text file similar in format to the input.

---

## General Rules

1. **Ignore timestamps**
   - Any leading timestamp like:
     ```
     [0| 3.712]
     ```
     must be ignored when determining uniqueness or changes.
   - The original timestamp must still be preserved in the output line if the line is kept.

2. **State-based deduplication (not line-based)**
   - A line should only be included if it represents a **change in state** compared to the last time that same state was observed.
   - Repeating the same state later in the log should be removed unless something meaningful changed in between.

3. **Do NOT group lines**
   - Each line is evaluated independently.
   - Do not collapse or compare entire blocks of lines together.
   - If only one line in a repeated block changes, only that line should reappear.

---

## Category-Specific Rules

### 1. `[PERCEPTION]` Lines

Examples:

[PERCEPTION] Calibrated cameras: ['front']
[PERCEPTION] Available cameras: ['front']


Rules:
- Treat the text **before the colon (`:`)** as the key.
- Treat the text **after the colon** as the value.
- Only emit a new line if the value changes.
- Repeated lines with identical lists (e.g. `['front']`) must be removed, even if they appear much later.

---

### 2. `[BASIC]` and `[ACIDIC]` Distance Lines

Examples:

[BASIC] id=140 REL dist=2569
[ACIDIC] id=102 REL dist=3096


Rules:
- Treat each object independently using this key:

[BASIC] id=<ID> REL dist
[ACIDIC] id=<ID> REL dist

- The **state value** is the numeric `dist`.

#### Distance Tolerance
- Ignore changes smaller than **20 units**.
- Only emit a new line if:

abs(new_dist - last_dist) >= 20


#### Implications
- Multiple `[BASIC]` or `[ACIDIC]` lines with different `id`s can appear together.
- When the same `id` appears again later:
- If the distance changed less than 20 → drop it
- If the distance changed ≥ 20 → keep it

---

### 3. Configuration Lines

Examples:

'wall_parallel_timeout_s': 4.0}


Rules:
- Treat the config key name as the state key.
- Only emit when the value changes.
- Ignore punctuation differences such as trailing semicolons or braces.

---

### 4. Other Bracketed Tags (e.g. `[LOC]`, `[NAV]`, etc.)

Examples:

[LOC] arena=3 pose_obs=YES


Rules:
- The bracket tag (e.g. `[LOC]`) defines the state category.
- The remainder of the line is the state value.
- Emit only when the value differs from the last seen value for that tag.
- If a value changes and later returns, all changes must be preserved:

YES → NO → YES (keep all three)


---

### 5. Logger-Style Lines

Examples:

sr.robot3.robot - INFO - Initialized successfully


Rules:
- Use `(logger_name + level)` as the state key.
- Use the message text as the value.
- Emit only on message change.

---

## Output Requirements

- Output must be a `.txt` file
- Original line formatting (timestamps, spacing, capitalization) must be preserved
- Only state changes should appear
- The file should be significantly shorter but semantically richer than the original log

---

## Summary

This is **state-change compression**, not text deduplication.

If nothing meaningful changed, the line must not appear — even if it reappears later in the log.





