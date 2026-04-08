3rdparty/RaspberryPi/README_RASPBERRYPI.md

# ⚠️ CRITICAL: Virtual Environment Setup

Picamera2 is installed via system packages (APT), not pip.

If you use a Python virtual environment, it MUST be created with:

```bash
python3 -m venv --system-site-packages ~/apriltag-env
```

Otherwise, you will get:

ModuleNotFoundError: No module named 'picamera2'

# Required setup

Activate environment:

```bash
source ~/apriltag-env/bin/activate
```
Verify:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

# Raspberry Pi 4B Setup & Deployment Guide

This document describes the full setup process for running the robot code on a Raspberry Pi 4B, including:

- Initial Pi setup
- Network and SSH access
- File transfer from desktop
- Python environment setup
- Installing dependencies
- Running the robot code
- Common issues and fixes

This is intended to be reproducible by someone else (or future you).

---

# 1. Base Raspberry Pi Setup

## OS Installation

- Install Raspberry Pi OS (64-bit recommended)
- Enable:
  - SSH
  - Camera interface

## System Update

```bash
sudo apt update
sudo apt upgrade -y
````

---

# 2. Enable Camera

Run:

```bash
sudo raspi-config
```

Then:

* Interface Options → Camera → Enable

Reboot:

```bash
sudo reboot
```

---

# 3. Network Setup (SSH Access)

Find the Pi IP:

```bash
hostname -I
```

Example:

```
192.168.8.236
```

---

## From Desktop → Connect to Pi

```bash
ssh bt@192.168.8.236
```

---

# 4. Project Directory Setup on Pi

Create working directory:

```bash
mkdir -p ~/robot
cd ~/robot
```

---

# 5. Copy Files from Desktop → Pi

⚠️ IMPORTANT: run this from your **desktop terminal**, not the Pi.

Correct pattern:

```bash
scp -r config bt@192.168.8.236:/home/bt/robot
scp -r calibration bt@192.168.8.236:/home/bt/robot
scp -r hw_io bt@192.168.8.236:/home/bt/robot
scp -r diagnostics bt@192.168.8.236:/home/bt/robot
scp main.py bt@192.168.8.236:/home/bt/robot
scp robot_controller.py bt@192.168.8.236:/home/bt/robot
```

### Common mistake

If you see:

```
scp: stat local "config": No such file or directory
```

You are in the wrong directory on your desktop. Fix by:

```bash
cd path/to/your/project
```

---

# 6. Python Environment Setup

Create virtual environment:

```bash
cd ~
python3 -m venv apriltag-env
```

Activate:

```bash
source ~/apriltag-env/bin/activate
```

Verify:

```bash
which python3
```

Expected:

```
/home/bt/apriltag-env/bin/python3

```

---

# 7. Install Dependencies

Inside the virtual environment:

```bash
pip install --upgrade pip
pip install numpy opencv-python
pip install pupil-apriltags
pip install picamera2
```

---

## Verify AprilTag Installation

```bash
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

Expected:

```
OK
```

---

# 8. Running the Robot Code

If you use a Python virtual environment, it MUST be created with:

```bash
python3 -m venv --system-site-packages ~/apriltag-env
```

Otherwise, you will get:

ModuleNotFoundError: No module named 'picamera2'

# Required setup

Activate environment:

```bash
source ~/apriltag-env/bin/activate
```
Verify:

```bash
python3 -c "from picamera2 import Picamera2; print('OK')"
python3 -c "from pupil_apriltags import Detector; print('OK')"
```

Run:

```bash
cd ~/robot
python3 main.py
```

---

# 9. Selecting Robot Mode

The system supports multiple robot profiles:

* `simulation` (uses SR API)
* `sr1` (real SR robot)
* `bob_bot` (custom Pi + Arduino robot)

Ensure configuration resolves correctly:

Example output:

```
hardware_profile: bob_bot
environment: real
```

---

## Important

If you see:

```
ModuleNotFoundError: No module named 'sr'
```

You are trying to run an SR robot without the SR API installed.

### Fix

Use:

```python
robot_id = "bob_bot"
hardware_profile = "bob_bot"
```

---

# 10. Running Without Motors (Camera Bring-Up)

If motors are not implemented yet, the system may fail with:

```
Level2.DRIVE: io.motors is not available
```

### Solution

Use diagnostics mode:

```python
RUN_MODE = RunMode.DIAGNOSTICS
```

Or temporarily stop before `controller.run()`.

---

# 11. Camera Setup

## Test Camera

```bash
python3 3rdparty/cameras/Pi3/test_camera.py
```

---

## Calibration

```bash
python3 3rdparty/cameras/Pi3/calibrate_pi_camera.py --capture
python3 3rdparty/cameras/Pi3/calibrate_pi_camera.py --solve
```

---

## AprilTag Test

```bash
python3 3rdparty/cameras/Pi3/apriltag_pi3_test.py --preview
```

---

# 12. Remote Execution Workflow

Typical workflow:

### On Desktop

```bash
scp -r project_files bt@PI_IP:/home/bt/robot
```

### On Pi

```bash
ssh bt@PI_IP
source ~/apriltag-env/bin/activate
cd ~/robot
python3 main.py
```

---

# 13. Common Errors & Fixes

## Error: `No module named pupil_apriltags`

Cause:

* wrong Python environment

Fix:

```bash
source ~/apriltag-env/bin/activate
```

---

## Error: `scp: stat local ...`

Cause:

* running from wrong directory

Fix:

* `cd` into correct project directory on desktop

---

## Error: `No module named 'sr'`

Cause:

* using SR profile without SR API

Fix:

* switch to `bob_bot` profile

---

## Error: `io.motors is not available`

Cause:

* motors not implemented yet

Fix:

* use diagnostics mode
* or stub outputs

---

## Error: camera shows 0 markers

Cause:

* no tags visible OR calibration missing

Check:

* lighting
* tag visibility
* calibration values

---

# 14. Final Working State

System is considered working when:

* SSH works
* files copy correctly via SCP
* virtual environment loads
* AprilTag library imports
* camera initializes
* `camera.see()` runs without crashing
* diagnostics mode prints detections

---

# 15. Next Steps

* Implement motor IO (Arduino / driver layer)
* Enable full autonomous run
* Add button-trigger start
* Add systemd auto-start (optional)

---

# Summary

This setup establishes:

* reliable remote access (SSH)
* reproducible deployment (SCP)
* isolated Python environment
* working camera + AprilTag detection
* ability to run robot code without full hardware

This forms the foundation for full robot operation.

```
