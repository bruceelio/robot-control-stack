📊 TRACE Logging System

The TRACE system provides structured, machine-parseable logs for every tick and every phase transition.

Implemented in:

log_trace.py


(Do not name this trace.py — conflicts with Python stdlib.)

🧾 TRACE Line Format

Each log line:

TRACE t=<sec> tick=<n> src=<component> evt=<event> run=<id> phase=<phase> key=value ...


Example:

TRACE t=1738150124.105 tick=44 src=ACQ evt=PHASE_ENTER run=7 phase=ALIGN lock=132


Fields:

Field	Meaning
t	wall-clock seconds
tick	controller tick counter
src	component emitting event
evt	event code
run	AcquireObject run id
phase	behavior phase
extra	structured key=value fields

No free text — always key=value.

🧩 TRACE Sources

Common src values:

ACQ       AcquireObject behavior
TRACK     TrackObject snapshot (emitted by caller)
SELECT    SelectTarget skill
ALIGN     AlignToTarget skill
APPROACH  ApproachTarget skill
RECOVER   RecoverLostTarget behavior
GSEARCH   GlobalSearch behavior

🧷 TRACE Event Types

Examples:

ACQ_START
PHASE_ENTER
TRACK_UPDATE
SELECT_PICK
LOCK_SET
MOTION_CMD
MOTION_DONE
RECOVER_RUNG_ENTER
RECOVER_DONE
ACQ_DONE


Rules:

PHASE_ENTER only on real transitions (not per tick)

TRACK_UPDATE once per tick

motion logged only at command + completion

throttled events for “waiting” states

⏱ Tick & Run Counters

log_trace.py maintains:

next_tick()  → called once per controller loop
next_run()   → called once per AcquireObject.start()


This enables:

multi-run separation

deterministic timelines

replay grouping

🔎 How to Read a TRACE Timeline

Minimal successful acquisition looks like:

ACQ_START
TRACK_UPDATE lock=none
SELECT_PICK id=…
LOCK_SET
PHASE_ENTER ALIGN
PHASE_ENTER APPROACHING
MOTION_CMD drive=…
TRACK_UPDATE dist decreasing
PHASE_ENTER GRABBING
ACQ_DONE SUCCEEDED


Recovery run shows:

VISION_LOSS_ESCALATE
PHASE_ENTER RECOVER_LOST_TARGET
RECOVER_RUNG_ENTER REACQUIRE
RECOVER_RUNG_DONE FAILED
RECOVER_RUNG_ENTER BACKOFF_SCAN
RECOVER_DONE LOCKED_RECOVERED
PHASE_ENTER TRACK_AFTER_RECOVER
PHASE_ENTER SELECT

🛠 Extension Rules

If you add new behaviors or skills:

✅ Do:

emit TRACE at phase transitions

keep one tracker update per tick

log motion commands & completions

use key=value fields only

❌ Don’t:

log inside primitives

log inside TrackObject

emit PHASE_ENTER inside per-tick loops

mix free text with TRACE lines

🧪 Testing & Debugging Strategy

TRACE logs are designed so you can:

reconstruct phase timelines

verify recovery ladders

measure stall durations

confirm lock stability

detect double motion commands

diff good vs bad runs

Logs are parseable into:

CSV

timelines

replay visualizers

grading scripts