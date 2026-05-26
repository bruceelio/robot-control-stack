# perf_monitor.py

import time


class PerformanceMonitor:
    def __init__(self, name: str, report_every_s: float = 5.0):
        self.name = name
        self.report_every_s = report_every_s

        self.window_start = time.perf_counter()
        self.last_report = self.window_start

        self.ticks = 0
        self.work_s = 0.0
        self.max_tick_s = 0.0

    def record_tick(self, work_s: float) -> None:
        now = time.perf_counter()

        self.ticks += 1
        self.work_s += work_s
        self.max_tick_s = max(self.max_tick_s, work_s)

        elapsed = now - self.window_start
        if elapsed < self.report_every_s:
            return

        rate_hz = self.ticks / elapsed
        avg_work_ms = (self.work_s / self.ticks) * 1000.0
        max_work_ms = self.max_tick_s * 1000.0
        util = self.work_s / elapsed

        print(
            f"[PERF][{self.name}] "
            f"rate={rate_hz:.1f}Hz "
            f"avg_work={avg_work_ms:.1f}ms "
            f"max_work={max_work_ms:.1f}ms "
            f"util={util * 100:.1f}%"
        )

        self.window_start = now
        self.ticks = 0
        self.work_s = 0.0
        self.max_tick_s = 0.0