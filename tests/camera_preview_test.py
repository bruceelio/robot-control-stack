# tests/camera_preview_test.py

import cv2

from hw_io.base import IOMap   # or however you initialise your IO

def main():
    io = IOMap()
    cam = io.cameras()["front"]

    while True:
        frame_rgb, markers = cam.see_with_frame()

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # draw markers (optional)
        for marker in markers:
            cx, cy = int(marker.center_px[0]), int(marker.center_px[1])
            cv2.circle(frame_bgr, (cx, cy), 5, (0, 0, 255), -1)

        cv2.imshow("Camera", frame_bgr)

        print(f"markers: {len(markers)}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()