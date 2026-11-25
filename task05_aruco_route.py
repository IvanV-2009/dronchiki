"""№ 5. Напольные маркеры в местах поворота. Курс корпуса постоянен."""
import math
import time
import settings as cfg
from vision import camera_mission, frame_from, floor_offset, local_offset, marker_reader, read_id


def turn(direction, degrees):
    a = math.radians(degrees)
    dx, dy = direction
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def main():
    finish = read_id("ID конечного маркера: ")
    markers = marker_reader()

    def route(flight, camera):
        direction = local_offset(0, 1)
        previous = None
        while True:
            frame = frame_from(camera)
            x, y, z = flight.here()
            candidates = []
            for marker, pixel in markers(frame).items():
                dx, dy = floor_offset(frame, pixel, z)
                # Пропускаем уже обработанную развилку, пока выезжаем с неё.
                if previous and marker == previous[0] and math.dist(
                        (x + dx, y + dy), previous[1]) < 0.40:
                    continue
                ahead = dx * direction[0] + dy * direction[1]
                side = abs(dx * direction[1] - dy * direction[0])
                if ahead >= -0.10 and side < 0.30:
                    candidates.append((math.hypot(dx, dy), marker, dx, dy))
            if candidates:
                distance, marker, dx, dy = min(candidates)
                if distance < 0.07:
                    flight.go(x + dx, y + dy, cfg.HEIGHT, tolerance=0.05)
                    print(f"Маркер {marker}")
                    if marker == finish:
                        return
                    angle = cfg.TURN_DEGREES if marker % 2 == 0 else -cfg.TURN_DEGREES
                    direction = turn(direction, angle)
                    previous = (marker, (x + dx, y + dy))
                    continue
                scale = min(1.0, 0.18 / distance)
                step_x, step_y = scale * dx, scale * dy
            else:
                step_x, step_y = 0.18 * direction[0], 0.18 * direction[1]
            flight.aim(x + step_x, y + step_y, cfg.HEIGHT)
            time.sleep(0.09)

    camera_mission(cfg.DOWN_CAMERA, route)


if __name__ == "__main__":
    main()
