from hw_io.clients.mega_client import MegaSerialClient, MegaSerialConfig
import time

cfg = MegaSerialConfig(port="/dev/ttyACM0", baud=115200, timeout=1.0, open_delay_s=2.0)
mega = MegaSerialClient(cfg)
mega.open()

print(mega.hello())
print(mega.mode_auto())

seq = 1
for _ in range(5):
    print(mega.heartbeat(seq)); seq += 1; time.sleep(0.05)

print(mega.servo_write(12, 1.0))   # lift up
for _ in range(10):
    print(mega.heartbeat(seq)); seq += 1; time.sleep(0.05)

print(mega.servo_write(12, -1.0))  # lift down
for _ in range(10):
    print(mega.heartbeat(seq)); seq += 1; time.sleep(0.05)

print(mega.servo_write(11, 1.0))   # grip one way
for _ in range(10):
    print(mega.heartbeat(seq)); seq += 1; time.sleep(0.05)

print(mega.servo_write(11, -1.0))  # grip other way
for _ in range(10):
    print(mega.heartbeat(seq)); seq += 1; time.sleep(0.05)

print(mega.stop())
print(mega.mode_teleop())
mega.close()