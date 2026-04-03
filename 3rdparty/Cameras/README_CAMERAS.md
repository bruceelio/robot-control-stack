3rdparty/cameras/Pi3/README.md

Great question — this is the key mental shift from Webots → real hardware.
Let’s walk it through clearly.
________________________________________
🧠 In Webots (what you’re used to)
When you do:
markers = robot.camera.see()
👉 The SR API:
•	already created the camera
•	already connected it to simulation
•	already returns fake (but realistic) data
You never see how it’s built.
________________________________________
⚙️ In your real robot stack
You built that pipeline yourself — and it’s actually very clean 👍
Here is the full chain:
________________________________________
🔗 The full flow (end-to-end)
1. main.py
controller = Controller(robot)
This is where everything begins.
________________________________________
2. Controller.__init__()
This is the critical step:
self.io: IOMap = resolve_io(
    robot=robot,
    hardware_profile=CONFIG.hardware_profile,
)
👉 This is where all hardware is created, including the camera.
________________________________________
3. hw_io/resolve.py
This decides:
if hardware_profile == "bob_bot":
    return BobBotIO(robot)
👉 You now get a BobBotIO object
________________________________________
4. BobBotIO.__init__()
Inside this (your implementation):
👉 it calls something like:
from hw_io.cameras.resolve import resolve_camera
self._cameras["front"] = resolve_camera("pi3")
________________________________________
5. hw_io/cameras/resolve.py
This is the key decision point:
if backend == "pi_libcamera_april":
    return PiLibcameraAprilCamera(...)
👉 This is where your real camera backend is chosen
________________________________________
6. PiLibcameraAprilCamera.__init__()
💥 THIS is where the camera actually starts
Inside your file:
self._picam2 = Picamera2()

config = self._picam2.create_preview_configuration(...)
self._picam2.configure(config)

self._picam2.start()
👉 This is the moment:
•	libcamera is initialised
•	sensor is opened
•	frames start streaming
________________________________________
🎯 So the answer is:
👉 The camera is initiated here:
self._picam2.start()
inside:
👉 PiLibcameraAprilCamera.__init__()
________________________________________
🔁 What happens when you call camera.see()
Later:
cam = io.cameras()["front"]
markers = cam.see()
________________________________________
Inside see()
frame_rgb = self.capture()
________________________________________
Inside capture()
return self._picam2.capture_array()
👉 This grabs a real frame from the sensor
________________________________________
Then:
detections = self._detector.detect(...)
👉 AprilTag detection runs
________________________________________
Then:
return [Marker(...), Marker(...)]
👉 You get SR-style marker objects
________________________________________
🧠 Compare to Webots
Step	Webots	Your system
Camera creation	Hidden in SR API	You build it in PiLibcameraAprilCamera
Frame source	Simulation	Real sensor via libcamera
Detection	SR internal	pupil_apriltags
Output	SR markers	Your Marker dataclass
________________________________________
🔥 Why your architecture is good
You now have:
camera.see()
that behaves like SR — but is:
•	real hardware
•	configurable
•	testable standalone
•	interchangeable backend
That’s exactly what you wanted.
________________________________________
⚠️ Important implication (you just hit this)
Because the camera is started in __init__():
👉 You must only create it once
That’s why earlier I said:
don’t call resolve_io() again in diagnostics
Otherwise you try to do:
Picamera2.start()  ← already running
…and libcamera will complain or fail.
________________________________________
🧪 Debug trick (very useful)
If you want to confirm when the camera starts, add:
print("[CAMERA] Starting Pi camera")
right before:
self._picam2.start()
Then you’ll see exactly when it happens.
________________________________________
🎯 Final mental model
Think of your system like this:
main.py
  ↓
Controller
  ↓
IOMap (resolve_io)
  ↓
Camera resolver
  ↓
PiLibcameraAprilCamera
  ↓
Picamera2.start()   ← REAL HARDWARE STARTS HERE
________________________________________
🚀 You are now here
You’ve successfully moved from:
“camera magically exists”
to:
“I fully control camera lifecycle, calibration, and detection”
That’s a big step.
________________________________________
If you want next step:
👉 I can show you how to log raw frames + detections to disk for calibration verification (super useful next step)


# Cameras

This directory contains camera backends, calibration workflows, and testing tools
for all supported camera types.

Each camera implementation should:
- provide a consistent `camera.see()` interface
- return SR-style marker data
- be independently testable outside the robot stack

---

# Architecture Overview

In the SR API (simulation or real kit):

```python
markers = robot.camera.see()
camera is created internally
detection is handled by the API
data is returned in SR format

In this system, the full pipeline is explicitly implemented:

main.py
  ↓
Controller
  ↓
IOMap (resolve_io)
  ↓
Camera resolver
  ↓
Camera backend (e.g. PiLibcameraAprilCamera)
  ↓
camera.see()

Each backend is responsible for:

capturing frames
running detection (e.g. AprilTags)
computing geometry (distance, bearing, etc.)
returning Marker objects
Camera Responsibilities

Every camera backend must:

return consistent marker objects
handle calibration internally
support standalone testing
not depend on robot motion or other subsystems
Calibration (Generic)

Calibration provides:

focal lengths (fx, fy)
optical center (cx, cy)

These are used to compute:

distance to tag
bearing
vertical angle

Calibration is:

camera-specific
resolution-specific
invalidated if focus or mounting changes
Testing Philosophy

There are two levels of testing:

1. Standalone Camera Testing

Run directly against the camera:

verify camera opens
verify detection works
verify geometry is reasonable
2. Integrated Diagnostics

Run through the robot stack:

verify resolver works
verify camera.see() integration
verify configuration pipeline
Multiple Camera Support

Each camera type should have its own directory:

cameras/
  README.md
  Pi3/
  C270/

Each camera folder contains:

setup instructions
calibration workflow
test scripts
Key Principle

camera.see() must return fully usable data.

No additional processing should be required in:

perception
controller
behaviors
Future Cameras

When adding a new camera:

Create a new folder (e.g. C270/)
Implement a backend
document setup and calibration
verify standalone + integrated tests