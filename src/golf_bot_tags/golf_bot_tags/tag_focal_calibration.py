"""Measure the camera focal length in pixels (needed for distance-from-size).

Put one tag straight in front of the camera at a measured distance (camera lens to tag),
then run:

  ros2 run golf_bot_tags tag_focal_calibration --distance 1.00 --size 0.15

It prints  focal_px = distance * tag_height_px / tag_size  averaged over ~3 s.
Put the result in tag_mission.yaml (focal_px). Repeat at 2-3 distances to check it.
"""

import argparse
import time

import cv2

from golf_bot_tags.tag_vision import TagDetector


def main(args=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('--distance', type=float, required=True, help='camera-to-tag distance (m)')
    ap.add_argument('--size', type=float, default=0.15, help='black square side length (m)')
    ap.add_argument('--family', default='36h11')
    ap.add_argument('--camera', type=int, default=0, help='video device index')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--seconds', type=float, default=3.0)
    a, _ = ap.parse_known_args(args)

    cap = cv2.VideoCapture(a.camera, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    if not cap.isOpened():
        raise SystemExit(f'Cannot open camera {a.camera}')
    det = TagDetector(a.family, a.size, focal_px=1.0)  # focal irrelevant: we use h_px only
    values, t0 = [], time.monotonic()
    while time.monotonic() - t0 < a.seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        for o in det.detect(frame, time.monotonic()):
            values.append(a.distance * o.h_px / a.size)
    cap.release()
    if not values:
        raise SystemExit('No tag detected. Check the tag family / lighting / distance.')
    values.sort()
    f = values[len(values) // 2]
    w = frame.shape[1]
    import math
    hfov = math.degrees(2 * math.atan(w / 2 / f))
    print(f'focal_px: {f:.1f}   (from {len(values)} detections, image width {w} px, '
          f'horizontal FOV {hfov:.1f} deg)')


if __name__ == '__main__':
    main()
