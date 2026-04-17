from hw_io.clients.mega_client import MegaSerialClient, MegaSerialConfig
import time

cfg = MegaSerialConfig(port="/dev/ttyACM0", baud=115200, timeout=1.0, open_delay_s=2.0)
mega = MegaSerialClient(cfg)
mega.open()

print("forward")
mega.link_18_19("M1", 0.4)
mega.link_18_19("M2", 0.4)
time.sleep(3.0)

print("stop")
mega.link_18_19("M1", 0.0)
mega.link_18_19("M2", 0.0)
mega.close()