# Documentation Style Guide

## Purpose

This document defines the documentation standard used throughout the project.

The objective is simple:

> Help the operator complete a task as quickly, accurately and confidently as possible.

Operator manuals are not textbooks. They are practical workshop guides.

---

# Core Principles

## 1. Write for the operator

Assume the reader wants to complete a task, not study the theory.

Explain:

- What to do
- What to expect
- How to verify success

Reference external material for theory whenever practical.

---

## 2. Write for the workshop

Assume the operator is:

- Standing beside the robot
- Looking between the robot and the monitor
- Copying commands
- Holding tools or hardware
- Reading from several feet away

Documentation should support quick glances rather than extended reading.

---

## 3. Keep it simple

Every sentence should earn its place.

Before adding text, ask:

> Does this help the operator complete the task?

If not, remove it.

---

## 4. Use descriptive titles

Titles should explain what the operator will achieve.

Good:

- Verify Camera Detection
- Adjust Camera Focus
- Capture Calibration Images

Avoid vague titles.

---

## 5. Prefer bullets

Use bullet lists wherever possible.

Bullets are easier to scan than paragraphs.

Good:

- Camera detected
- Live image displayed
- Chessboard detected

Avoid long procedural paragraphs.

---

## 6. Keep commands separate

Commands should always be easy to copy and paste.

Example:

```bash
python camera-focus.py
```

Never hide commands inside paragraphs.

---

## 7. Use checklists for verification

Verification should answer one question:

Did it work?

Example:

- Camera detected
- Image displayed
- No errors
- Chessboard found

---

## 8. Minimise memory

Never expect the operator to remember information from earlier chapters.

If something is needed:

- show the command
- show the filename
- show the keyboard shortcuts
- show the recommended setting

where it is needed.

---

## 9. Use sections only when they add value

Typical step:

- Run
- Procedure
- Verification

Additional sections are optional.

Examples:

- Keyboard Controls
- Output Files
- Troubleshooting
- Notes
- References

Do not include empty sections.

---

## 10. Use numbering only when order matters

Numbered lists:

1. Activate environment
2. Run script
3. Save results

Bullet lists:

- Close
- Far
- Left
- Right
- Rotated

---

## 11. Prefer references over repetition

Do not reproduce large amounts of theory.

Instead, reference:

- Official documentation
- Videos
- Engineering references
- Internal design documents

---

## 12. Show, don't describe

Where practical:

- screenshots
- diagrams
- examples
- command output

are preferable to lengthy explanations.

Only include images that genuinely help the operator.

---

# Standard Step Structure

Use only the sections required for the task.

Typical example:

## Step X.X – Descriptive Title

### Run

```bash
python script.py
```

### Procedure

- Action
- Action
- Action

### Verification

- Result
- Result
- Result

Optional:

- Keyboard Controls
- Output Files
- Notes
- Troubleshooting
- References

---

# Documentation Checklist

Before publishing, ask:

- Is every heading meaningful?
- Can commands be copied easily?
- Can procedures be followed from several feet away?
- Can verification be completed quickly?
- Have unnecessary paragraphs been removed?
- Have repeated explanations been removed?
- Would a first-time student know what to do next?

If the answer is yes, the document is ready.