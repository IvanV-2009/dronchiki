"""№ 4. Следуем по чёрной замкнутой линии до команды Esc."""
import math
import time

import cv2
import numpy as np
import settings as cfg
from vision import camera_mission, floor_offset, frame_from


def ahead_on_line(frame, direction):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, ink = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    h, w = ink.shape
    if cv2.countNonZero(ink) > 0.65 * h * w:
        return None   # Кадр закрыт или фон слишком тёмный.
    cx, cy = w // 2, h // 2
    radius = int(min(h, w) * 0.27)
    # Берём компоненту линии, ближайшую к точке под дроном.
    count, labels, stats, _ = cv2.connectedComponentsWithStats(ink)
    best = None
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < 0.001 * w * h:
            continue
        ys, xs = np.where(labels == i)
        distance = np.min((xs - cx)**2 + (ys - cy)**2)
        if best is None or distance < best[0]:
            best = distance, i
    if best is None or best[0] > (radius * 0.75)**2:
        return None
    component = (labels == best[1]).astype(np.uint8) * 255
    # Линия пересекает окружность впереди и сзади. Проекция позволяет
    # выбрать продолжение движения и не развернуться на следующем кадре.
    ring = np.zeros_like(ink)
    cv2.circle(ring, (cx, cy), radius, 255, max(3, radius // 12))
    arcs, _ = cv2.findContours(ring & component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    points = []
    for arc in arcs:
        m = cv2.moments(arc)
        if m["m00"] > 2:
            points.append((m["m10"]/m["m00"], m["m01"]/m["m00"]))
    if points:
        return max(points, key=lambda p: (p[0]-cx)*direction[0] + (p[1]-cy)*direction[1])
    return None


def follow(flight, camera):
    direction = (0, -1)
    missing_since = None
    while True:
        try:
            frame = frame_from(camera)
        except KeyboardInterrupt:
            flight.hold(flight.here(), seconds=0.4)
            return
        point = ahead_on_line(frame, direction)
        x, y, z = flight.here()
        if point is None:
            if missing_since is None:
                missing_since = time.monotonic()
                flight.aim(x, y, z)
            if time.monotonic() - missing_since > 1.5:
                raise RuntimeError("Линия вышла из поля зрения.")
        else:
            missing_since = None
            h, w = frame.shape[:2]
            direction = (point[0] - w / 2, point[1] - h / 2)
            dx, dy = floor_offset(frame, point, z)
            gain = min(1.0, 0.22 / max(math.hypot(dx, dy), 0.001))
            flight.aim(x + gain*dx, y + gain*dy, cfg.HEIGHT)
        time.sleep(0.08)




def preview_camera(camera_index=0):

    # Подключаем камеру
    camera = cv2.VideoCapture(camera_index)

    if not camera.isOpened():
        raise RuntimeError("Не удалось подключить камеру")

    # Начальное направление поиска
    direction = (0, -1)

    try:
        while True:

            # Получаем кадр
            success, frame = camera.read()

            if not success:
                print("Не удалось получить изображение")
                break

            h, w = frame.shape[:2]

            # Центр изображения
            cx, cy = w // 2, h // 2

            # Радиус поиска
            radius = int(min(h, w) * 0.27)

            # Вызываем твой алгоритм
            point = ahead_on_line(frame, direction)

            # Создаём копию для визуализации
            display = frame.copy()

            # Рисуем центр камеры
            cv2.drawMarker(
                display,
                (cx, cy),
                (0, 255, 255),
                cv2.MARKER_CROSS,
                20,
                2
            )

            # Рисуем окружность поиска
            cv2.circle(
                display,
                (cx, cy),
                radius,
                (255, 0, 0),
                2
            )

            if point is not None:

                px = int(round(point[0]))
                py = int(round(point[1]))

                # Вектор от центра к найденной точке
                dx_px = point[0] - cx
                dy_px = point[1] - cy

                # Обновляем направление поиска
                direction = (dx_px, dy_px)

                # Показываем найденную точку
                cv2.circle(
                    display,
                    (px, py),
                    9,
                    (0, 0, 255),
                    -1
                )

                # Рисуем направление от центра
                cv2.arrowedLine(
                    display,
                    (cx, cy),
                    (px, py),
                    (0, 255, 0),
                    2
                )

                # Выводим координаты
                cv2.putText(
                    display,
                    f"Point: ({px}, {py})",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                cv2.putText(
                    display,
                    f"Offset: ({dx_px:.1f}, {dy_px:.1f}) px",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                print(
                    f"Point: ({px}, {py}) | "
                    f"Offset: ({dx_px:.1f}, {dy_px:.1f}) px"
                )

            else:

                cv2.putText(
                    display,
                    "LINE NOT FOUND",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            # Отдельно показываем бинарную маску
            gray = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2GRAY
            )

            _, mask = cv2.threshold(
                gray,
                0,
                255,
                cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU
            )

            cv2.imshow("Camera - Line Detection", display)
            cv2.imshow("Binary Mask", mask)

            # Выход по клавише Q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    preview_camera(0)


