# tests/test_buzzer_patterns.py

from hw_io.buzzer_patterns import BuzzerPatterns, BuzzerCue


class FakeLevel2:
    def __init__(self):
        self.calls = []  # list of tuples for assertions

    def BUZZ(self, tone, duration, *, blocking=False):
        self.calls.append(("BUZZ", tone, float(duration), bool(blocking)))

    def SLEEP(self, secs: float):
        self.calls.append(("SLEEP", float(secs)))


def test_buzzer_cue_start():
    l2 = FakeLevel2()
    p = BuzzerPatterns(l2)

    p.cue(BuzzerCue.START)

    # Expect: BUZZ, SLEEP, BUZZ (no trailing sleep because last gap is 0)
    assert l2.calls[0][0] == "BUZZ"
    assert l2.calls[1][0] == "SLEEP"
    assert l2.calls[2][0] == "BUZZ"
    assert len(l2.calls) == 3


def test_buzzer_cue_error():
    l2 = FakeLevel2()
    p = BuzzerPatterns(l2)

    p.cue(BuzzerCue.ERROR)

    # Expect: BUZZ, SLEEP, BUZZ (last gap is 0)
    assert [c[0] for c in l2.calls] == ["BUZZ", "SLEEP", "BUZZ"]


def test_buzzer_rickroll_intro_has_multiple_steps():
    l2 = FakeLevel2()
    p = BuzzerPatterns(l2)

    p.rickroll_intro()

    buzz_calls = [c for c in l2.calls if c[0] == "BUZZ"]
    assert len(buzz_calls) >= 7
