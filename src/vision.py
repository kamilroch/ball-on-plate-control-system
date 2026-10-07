import math
import time

import cv2
import numpy as np
from picamera2 import Picamera2


WIDTH = 960
HEIGHT = 720

CENTER_X = 480
CENTER_Y = 300

LOWER = np.array(
    [94, 250, 245],
    dtype=np.uint8,
)
UPPER = np.array(
    [108, 255, 255],
    dtype=np.uint8,
)

VISION_MIN_AREA = 700.0
VISION_MAX_AREA = 12000.0

VISION_MIN_RADIUS = 20.0
VISION_MAX_RADIUS = 80.0

VISION_MIN_CIRCULARITY = 0.20
VISION_MAX_ASPECT = 4.0

KERNEL_OPEN = np.ones(
    (5, 5),
    np.uint8,
)
KERNEL_CLOSE = np.ones(
    (5, 5),
    np.uint8,
)


def create_camera():
    camera = Picamera2()

    config = camera.create_preview_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888",
        },
        buffer_count=4,
    )

    camera.configure(config)
    camera.start()

    try:
        camera.set_controls(
            {
                "FrameDurationLimits": (
                    33333,
                    33333,
                )
            }
        )
    except Exception:
        pass

    time.sleep(1.0)
    return camera


def detect_ball_hsv94(frame_rgb):
    hsv = cv2.cvtColor(
        frame_rgb,
        cv2.COLOR_RGB2HSV,
    )

    mask = cv2.inRange(
        hsv,
        LOWER,
        UPPER,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        KERNEL_OPEN,
        iterations=1,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        KERNEL_CLOSE,
        iterations=1,
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []

    for contour in contours:
        area = float(
            cv2.contourArea(contour)
        )

        x_box, y_box, w_box, h_box = (
            cv2.boundingRect(contour)
        )

        touches_edge = (
            x_box <= 2
            or y_box <= 2
            or x_box + w_box >= WIDTH - 2
            or y_box + h_box >= HEIGHT - 2
        )

        if touches_edge:
            min_area = 250.0
            max_area = 12000.0
            min_radius = 10.0
            max_radius = 85.0
            min_circularity = 0.08
            max_aspect = 3.5
        else:
            min_area = VISION_MIN_AREA
            max_area = VISION_MAX_AREA
            min_radius = VISION_MIN_RADIUS
            max_radius = VISION_MAX_RADIUS
            min_circularity = VISION_MIN_CIRCULARITY
            max_aspect = VISION_MAX_ASPECT

        if not min_area <= area <= max_area:
            continue

        if w_box < 8 or h_box < 8:
            continue

        aspect = max(
            w_box / max(h_box, 1),
            h_box / max(w_box, 1),
        )

        if aspect > max_aspect:
            continue

        perimeter = float(
            cv2.arcLength(contour, True)
        )

        if perimeter <= 1e-9:
            continue

        circularity = (
            4.0
            * math.pi
            * area
            / (perimeter * perimeter)
        )

        if circularity < min_circularity:
            continue

        (x, y), radius = (
            cv2.minEnclosingCircle(contour)
        )
        radius = float(radius)

        if not min_radius <= radius <= max_radius:
            continue

        radius_score = (
            -abs(radius - 52.0) * 0.10
        )

        score = (
            circularity * 8.0
            + min(area, 9000.0) / 3000.0
            + radius_score
        )

        if touches_edge:
            score -= 0.35

        candidates.append(
            {
                "x": float(x),
                "y": float(y),
                "r": radius,
                "area": area,
                "circ": circularity,
                "edge": touches_edge,
                "score": score,
                "source": "HSV94",
            }
        )

    if not candidates:
        return None, mask

    best = max(
        candidates,
        key=lambda candidate: candidate["score"],
    )

    return best, mask
