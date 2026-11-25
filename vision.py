"""Камера, ArUco и перевод пикселей в метры на плоском полу."""
import math
import time
from contextlib import contextmanager

import cv2
import numpy as np
from pioneer_sdk2 import Camera, CameraType, CONFIG
import settings as cfg
from flight import Flight, airborne, check_point, connection


@contextmanager
def camera_view(camera_name):
    kind = CameraType[camera_name]
    if CONFIG["cameras"]["driver"] == "rtsp":
        camera = Camera(kind, camera_ip=cfg.CAMERA_IP)
    else:
        camera = Camera(kind)
    try:
        yield camera
    finally:
        camera.stop()
        cv2.destroyAllWindows()


def frame_from(camera):
    frame = camera.get_cv_frame(timeout=1.0)
    if frame is None or not frame.size:
        raise RuntimeError("Пустой кадр.")
    turns = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180,
             270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    if cfg.IMAGE_ROTATION:
        frame = cv2.rotate(frame, turns[cfg.IMAGE_ROTATION])
    if frame.ndim == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    cv2.imshow("Pioneer | Esc = land", frame)
    if (cv2.waitKey(1) & 255) == 27:
        raise KeyboardInterrupt
    return frame


def camera_mission(camera_name, action):
    check_point(0, 0, cfg.HEIGHT)
    with camera_view(camera_name) as camera:
        frame_from(camera)  # Проверим видеопоток до запуска моторов.
        with connection() as drone:
            flight = Flight(drone)
            with airborne(flight):
                flight.go(0, 0, cfg.HEIGHT)
                action(flight, camera)


def focal_lengths(frame):
    height, width = frame.shape[:2]
    fx = width / (2 * math.tan(math.radians(cfg.HFOV / 2)))
    fy = height / (2 * math.tan(math.radians(cfg.VFOV / 2)))
    return fx, fy


def local_offset(right, forward):
    """При постоянном курсе переводим вправо/вперёд в локальные X/Y."""
    a = math.radians(cfg.FORWARD_ANGLE)
    return (forward * math.cos(a) + right * math.sin(a),
            forward * math.sin(a) - right * math.cos(a))


def floor_offset(frame, pixel, height):
    h, w = frame.shape[:2]
    fx, fy = focal_lengths(frame)
    right = (pixel[0] - w / 2) * height / fx
    forward = (h / 2 - pixel[1]) * height / fy
    return local_offset(right, forward)


def grid(low, high, step):
    count = max(1, math.ceil((high - low) / step))
    return np.linspace(low, high, count + 1)


def search_points():
    yield 0, 0
    xs = grid(*cfg.X_LIMITS, cfg.SEARCH_STEP)
    for row, y in enumerate(grid(*cfg.Y_LIMITS, cfg.SEARCH_STEP)):
        for x in (xs if row % 2 == 0 else xs[::-1]):
            yield x, y


COLOR_RANGES = {
    "красный": [(0, 10), (170, 179)], "оранжевый": [(11, 19)],
    "жёлтый": [(20, 35)], "зелёный": [(36, 86)], "синий": [(91, 135)],
}


def color_name(value):
    name = value.strip().lower().replace("желт", "жёлт").replace("зелен", "зелён")
    if name not in COLOR_RANGES:
        raise ValueError("Доступные цвета: " + ", ".join(COLOR_RANGES))
    return name


def color_mask(frame, color):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in COLOR_RANGES[color_name(color)]:
        mask |= cv2.inRange(hsv, (low, 90, 65), (high, 255, 255))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def center(contour):
    m = cv2.moments(contour)
    return m["m10"] / m["m00"], m["m01"] / m["m00"]


def marker_reader():
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.ARUCO_DICT))
    detector = cv2.aruco.ArucoDetector(dictionary)

    def read(frame):
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is None:
            return {}
        return {int(i): tuple(c.reshape(4, 2).mean(axis=0))
                for i, c in zip(ids.flatten(), corners)}
    return read


def read_id(prompt):
    value = int(input(prompt))
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, cfg.ARUCO_DICT))
    if not 0 <= value < len(dictionary.bytesList):
        raise ValueError(f"ID вне диапазона словаря {cfg.ARUCO_DICT}.")
    return value


def land_on_target(flight, camera, detect, check_pad=None):
    """detect возвращает центр посадочной цели в кадре или None."""
    for x, y in search_points():
        flight.go(x, y, cfg.HEIGHT)
        if detect(frame_from(camera)) is not None:
            break
    else:
        raise RuntimeError("Область поиска пройдена, посадочная цель не найдена.")

    steady = 0
    missing_since = None
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        frame = frame_from(camera)
        pixel = detect(frame)
        x, y, z = flight.here()
        if pixel is None:
            steady = 0
            if missing_since is None:
                missing_since = time.monotonic()
                flight.aim(x, y, z)
            if time.monotonic() - missing_since > 1.5:
                raise RuntimeError("Посадочная цель потеряна.")
            continue
        missing_since = None
        dx, dy = floor_offset(frame, pixel, z)
        error = math.hypot(dx, dy)
        if error > cfg.LAND_ERROR:
            steady = 0
            gain = min(0.75, 0.25 / error)
            flight.go(x + dx * gain, y + dy * gain, z, tolerance=0.025)
        else:
            steady += 1
            if steady >= 4:
                next_z = max(cfg.LAND_FROM, z - 0.18)
                low_enough = z <= cfg.LAND_FROM + 0.04
                if check_pad is None:
                    if low_enough:
                        return
                else:
                    fits, visible_lower = check_pad(frame, z, next_z)
                    if low_enough or not visible_lower:
                        if not fits:
                            raise RuntimeError('Опора не помещается в видимую площадку.')
                        return
                flight.go(x, y, next_z, tolerance=0.035)
                steady = 0
    raise TimeoutError("Не удалось устойчиво выровняться над целью.")
