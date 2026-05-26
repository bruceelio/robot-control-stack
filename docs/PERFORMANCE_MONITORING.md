# Performance Monitor Strategy

This document describes the performance monitor strategy for the robot control system.

The goal is not to print every frame or every tick. The goal is to collect lightweight timing data over a rolling window and periodically print useful summaries such as loop rate, work time, maximum stall time, and utilization.

---

## Why We Need This

The robot has several timing-sensitive subsystems:

- main controller loop
- perception pipeline
- camera worker processes
- localisation arbitration
- motion backend
- behaviour state machine

Without a performance monitor, it is hard to tell whether the robot is slow because of:

- AprilTag detection
- camera process latency
- blocking timed motion
- localisation work
- behaviour logic
- hardware IO calls
- serial/MEGA communication

The performance monitor gives a compact summary every few seconds so that we can see where time is actually being spent.

---

## Design Principle

Use one monitor per major loop or worker.

Recommended monitors:

```text
main
vision:front
vision:left      # future
vision:right     # future
vision:rear      # future
```

The main process monitor measures controller-loop health.

The vision monitor measures each camera process independently.

This matters because the main process and the camera workers are separate execution paths. A slow main loop does not necessarily mean the camera worker is slow, and a slow camera worker does not necessarily mean the main controller is blocked.

---

## Current Main Loop Monitor

The main controller should wrap each `Controller.update()` call:

```python
def update(self):
    tick_start = time.perf_counter()

    try:
        self._update_impl()
    finally:
        self.perf.record_tick(time.perf_counter() - tick_start)
```

The existing update body should live in:

```python
def _update_impl(self):
    ...
```

The monitor then prints every few seconds:

```text
[PERF][main] rate=6.2Hz avg_work=161.2ms max_work=3195.2ms util=100.0%
```

---

## Metric Meanings

### `rate`

Example:

```text
rate=19.8Hz
```

This is how many completed loop iterations happened per second during the reporting window.

For the main loop, this is the effective controller update rate.

Interpretation:

```text
~20Hz     good normal controller rate
10-15Hz   degraded but possibly usable
<10Hz     main loop is slow or blocked
```

If the robot is doing blocking timed movement, the main loop rate may drop heavily.

---

### `avg_work`

Example:

```text
avg_work=34.2ms
```

This is the average time spent inside one loop iteration.

For the main controller, it includes:

- encoder update
- perception readout
- localisation estimate
- behaviour update
- motion command calls
- any blocking calls triggered by behaviour or motion

A normal non-blocking main loop should usually have a modest average work time.

If this rises significantly, the main controller is spending too much time per tick.

---

### `max_work`

Example:

```text
max_work=3195.2ms
```

This is the longest single loop iteration during the reporting window.

This is often the most important value.

A large `max_work` means the controller was blocked for a long time.

Example from testing:

```text
max_work=3195ms
```

This matched a blocking sequence of:

```text
~0.5s timed reverse
~2.0s timed rotate
hardware overhead
```

So `max_work` revealed that timed motion currently blocks the main controller loop.

---

### `util`

Example:

```text
util=67.0%
```

This is the fraction of the reporting window spent doing work.

Formula:

```text
util = total_work_time / elapsed_window_time
```

Interpretation for the main loop:

```text
<50%      healthy headroom
50-80%    busy but probably usable
80-100%   little headroom
100%      loop is continuously busy or blocked
```

Important: `util=100%` does not necessarily mean CPU is at 100%. It means the measured loop spent the entire window inside its own work function. Blocking sleeps count as work from the loop's point of view because the controller cannot update during that time.

---

## Main Loop Example

Example output:

```text
[PERF][main] rate=6.2Hz avg_work=161.2ms max_work=3195.2ms util=100.0%
```

Meaning:

```text
rate=6.2Hz
```

The main controller only completed about 6 updates per second.

```text
avg_work=161.2ms
```

Average loop work was high.

```text
max_work=3195.2ms
```

At least one controller tick blocked for about 3.2 seconds.

```text
util=100.0%
```

The controller had no idle headroom in that window.

Likely cause:

```text
blocking timed drive / rotate / Level2.SLEEP
```

---

## Vision Worker Monitor

The camera worker should have its own monitor per camera.

Example:

```python
vision_perf = PerformanceMonitor(f"vision:{camera_name}", report_every_s=5.0)
```

Wrap the worker's per-frame processing:

```python
frame_start = time.perf_counter()

# capture frame
# detect AprilTags / markers
# build observations
# publish latest result

vision_perf.record_tick(time.perf_counter() - frame_start)
```

Expected output:

```text
[PERF][vision:front] rate=18.7Hz avg_work=31.5ms max_work=84.2ms util=58.9%
```

---

## Vision Metric Meanings

### `rate` for vision

This is effective processed frames per second.

Example:

```text
rate=18.7Hz
```

The camera worker processed about 18.7 frames per second.

This may be lower than camera capture FPS if AprilTag detection or image processing is expensive.

---

### `avg_work` for vision

Average processing time per camera frame.

Includes whatever is inside the monitored frame section:

- frame acquisition if included
- image conversion
- AprilTag detection
- observation building
- result publishing

Example:

```text
avg_work=31.5ms
```

This means one frame takes about 31.5 ms of worker time on average.

---

### `max_work` for vision

Worst frame processing time during the window.

Example:

```text
max_work=84.2ms
```

This means at least one frame took 84.2 ms.

This can indicate:

- occasional camera stall
- garbage collection pause
- detection complexity spike
- CPU contention
- OS scheduling delay

---

### `util` for vision

Vision worker occupancy.

Example:

```text
util=58.9%
```

This means the vision worker spent about 59% of the reporting window actively processing frames.

Interpretation:

```text
<60%      good headroom
60-80%    busy but usable
80-95%    near saturation
>95%      likely frame drops or latency buildup
```

---

## Important Distinction: Main Utilization vs CPU Utilization

The monitor does not directly measure operating-system CPU usage.

It measures loop occupancy.

That is usually more useful for robotics.

Example:

```text
[PERF][main] util=100%
```

This can happen even if CPU is not fully used, because the main loop may be blocked in:

```text
Level2.SLEEP
serial IO
blocking timed motion
hardware wait
```

From the robot control perspective, that is still important because the main loop cannot update while blocked.

---

## What Good Looks Like

Healthy main loop during normal sensing:

```text
[PERF][main] rate=19.5Hz avg_work=20.0ms max_work=70.0ms util=39.0%
```

Healthy vision worker:

```text
[PERF][vision:front] rate=18.0Hz avg_work=30.0ms max_work=60.0ms util=54.0%
```

Problem main loop during blocking motion:

```text
[PERF][main] rate=6.2Hz avg_work=161.2ms max_work=3195.2ms util=100.0%
```

Problem vision worker near saturation:

```text
[PERF][vision:front] rate=8.0Hz avg_work=120.0ms max_work=240.0ms util=96.0%
```

---

## Recommended Next Metrics

The basic monitor is enough to find major bottlenecks.

Later, add these:

### Vision result age

Time between when the camera result was produced and when the main loop consumes it.

```text
vision_age_ms=72
```

This tells whether the robot is acting on stale vision.

---

### Vision latency

Time between frame capture and observation publication.

```text
vision_latency_ms=95
```

This measures camera pipeline delay.

---

### Main loop state label

Include current robot state in performance output:

```text
state=GRABBING
```

Useful because blocking may only happen in certain behaviours.

---

### Motion blocking marker

When a timed motion begins, log:

```text
[MOTION_BLOCK] kind=drive duration=1.858s
```

This makes it easier to correlate performance spikes with movement.

---

## Current Known Finding

The first main-loop performance test showed:

```text
[PERF][main] rate=6.2Hz avg_work=161.2ms max_work=3195.2ms util=100.0%
```

This indicates the main controller loop is blocked during timed motion.

The likely cause is:

```text
TimedMotionBackend -> Level2.DRIVE -> Level2.SLEEP
```

This does not necessarily mean the camera worker is slow. It means the main controller is not updating while blocking motion is running.

To determine camera performance, add a separate `PerformanceMonitor` inside each camera worker process.

---

## Strategy Summary

Use performance monitoring in layers:

```text
main process monitor
    tells whether the controller loop is healthy

vision worker monitor per camera
    tells whether camera processing is healthy

vision age / latency
    tells whether main is consuming fresh or stale vision
```

Do not tune performance based on one metric alone.

Use:

```text
rate
avg_work
max_work
util
```

together.

For robotics, `max_work` and result age are often more important than average speed.
