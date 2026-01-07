# robot.py
from sr.robot import Robot             # official SR API
from localisation import Localisation # import your localisation class

def main():
    robot = Robot()               # SR robot instance
    localisation = Localisation(robot)

    print("Starting robot program...")

    while True:
        # Get all visible markers from the camera
        markers = robot.camera.see()

        # Estimate robot pose (x_mm, y_mm, heading_deg)
        pose = localisation.estimate_pose(markers)

        if pose is not None:
            x, y, heading = pose
            print(f"Robot position: x={x:.1f} mm, y={y:.1f} mm, heading={heading}")
        else:
            print("No markers visible, pose unknown")

        # --- Here you can add movement or other logic ---
        # For example: robot.drive.speed = 50

        # Optional small delay to avoid flooding output
        import time
        time.sleep(0.5)

if __name__ == "__main__":
    main()
