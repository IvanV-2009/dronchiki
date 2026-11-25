"""№ 6. Выбор цветной рамки, выравнивание по её проёму и пролёт."""
import math
import time

import cv2
import settings as cfg
from flight import check_point
from vision import camera_mission, color_mask, color_name, focal_lengths, frame_from, grid, local_offset


def opening(frame, color):
    mask = color_mask(frame, color)
    h, w = mask.shape
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return None
    fx, fy = focal_lengths(frame)
    candidates = []
    for i, contour in enumerate(contours):
        # У отверстия есть родитель — цветная рамка. Сплошная табличка не подойдёт.
        if hierarchy[0, i, 3] < 0 or cv2.contourArea(contour) < 0.008 * h * w:
            continue
        corners = cv2.approxPolyDP(contour, 0.025 * cv2.arcLength(contour, True), True)
        if len(corners) != 4 or not cv2.isContourConvex(corners):
            continue
        x, y, pw, ph = cv2.boundingRect(corners)
        if x < 2 or y < 2 or x + pw >= w - 1 or y + ph >= h - 1:
            continue
        if cv2.contourArea(corners) / (pw * ph) < 0.85:
            continue
        # Две независимые оценки расстояния до фронтальной рамки.
        depth_w = cfg.GATE_WIDTH * fx / pw
        depth_h = cfg.GATE_HEIGHT * fy / ph
        if abs(depth_w - depth_h) > 0.15 * min(depth_w, depth_h):
            continue
        depth = (depth_w + depth_h) / 2
        right = (x + pw / 2 - w / 2) * depth / fx
        up = (h / 2 - y - ph / 2) * depth / fy
        candidates.append((depth, right, up))
    return min(candidates) if candidates else None


def lateral_search():
    # Параллельно плоскости ворот, на неизменной стартовой дистанции.
    rx, ry = local_offset(1, 0)
    lower, upper = -math.inf, math.inf
    for component, bounds in ((rx, cfg.X_LIMITS), (ry, cfg.Y_LIMITS)):
        if abs(component) > 1e-8:
            a, b = sorted(v / component for v in bounds)
            lower, upper = max(lower, a), min(upper, b)
    yield 0, 0
    for offset in grid(lower, upper, cfg.SEARCH_STEP):
        yield rx * offset, ry * offset


def main():
    color = color_name(input("Цвет ворот (красный, оранжевый, жёлтый, зелёный, синий): "))
    side_room = (cfg.GATE_WIDTH - cfg.DRONE_WIDTH) / 2 - cfg.GATE_MARGIN
    top_room = (cfg.GATE_HEIGHT - cfg.DRONE_HEIGHT) / 2 - cfg.GATE_MARGIN
    if min(side_room, top_room) <= 0:
        raise ValueError("Недостаточно места для дрона в указанном проёме.")

    def fly(flight, camera):
        for x, y in lateral_search():
            flight.go(x, y, cfg.HEIGHT)
            if opening(frame_from(camera), color):
                break
        else:
            raise RuntimeError("На стартовой линии нужная рамка не найдена.")

        steady = 0
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            gate = opening(frame_from(camera), color)
            if gate is None:
                raise RuntimeError("Контур проёма потерян до пролёта.")
            depth, right, up = gate
            x, y, z = flight.here()
            if abs(right) > min(0.045, side_room) or abs(up) > min(0.045, top_room):
                steady = 0
                dx, dy = local_offset(max(-0.25, min(0.25, right)), 0)
                flight.go(x + dx, y + dy, z + max(-0.18, min(0.18, up)), tolerance=0.03)
            elif depth > cfg.STAGING_DISTANCE + 0.12:
                steady = 0
                dx, dy = local_offset(0, min(0.30, depth - cfg.STAGING_DISTANCE))
                flight.go(x + dx, y + dy, z, tolerance=0.05)
            else:
                steady += 1
                if steady < 4:
                    continue
                # Когда рамка выйдет за края кадра, цель уже за её плоскостью.
                dx, dy = local_offset(right, depth + cfg.BEYOND_GATE)
                target = (x + dx, y + dy, z + up)
                check_point(*target)
                flight.go(*target, tolerance=0.06)
                flight.hold(target, seconds=0.7, tolerance=0.09)
                return
        raise TimeoutError("Не удалось выровняться перед воротами.")

    camera_mission(cfg.FRONT_CAMERA, fly)


if __name__ == "__main__":
    main()
