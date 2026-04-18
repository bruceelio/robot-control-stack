import time
from hw_io.clients.mega_client import MegaSerialClient, MegaSerialConfig

def hb_for(mega, secs, seq):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        print(mega.heartbeat(seq))
        seq += 1
        time.sleep(0.1)
    return seq

cfg = MegaSerialConfig(port="/dev/ttyACM0", baud=115200, timeout=1.0, open_delay_s=2.0)
mega = MegaSerialClient(cfg)
mega.open()

print(mega.hello())
print(mega.mode_auto())

seq = 1
print(mega.heartbeat(seq)); seq += 1

print("forward main-like")
print(mega.link_18_19("M1", 0.6))
print(mega.link_18_19("M2", 0.6))
seq = hb_for(mega, 0.4324, seq)

print(mega.link_18_19("M1", 0.0))
print(mega.link_18_19("M2", 0.0))
seq = hb_for(mega, 0.2, seq)

print("rotate main-like")
print(mega.link_18_19("M1", 0.5))
print(mega.link_18_19("M2", -0.5))
seq = hb_for(mega, 0.2496, seq)

print(mega.link_18_19("M1", 0.0))
print(mega.link_18_19("M2", 0.0))
seq = hb_for(mega, 0.2, seq)

print(mega.stop())
print(mega.mode_teleop())
mega.close()