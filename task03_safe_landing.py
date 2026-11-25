"""№ 3. Поиск яркой площадки и посадка по центру её контура."""
import cv2
import settings as cfg
from vision import camera_mission, center, color_mask, focal_lengths, land_on_target


def pad_contour(frame):
    mask = color_mask(frame, cfg.PAD_COLOR)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    h, w = mask.shape
    candidates = []
    for contour in contours:
        if cv2.contourArea(contour) < 0.004 * h * w:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        # Когда рамка кадра режет площадку, видимый центр смещён.
        if x > 1 and y > 1 and x + width < w - 1 and y + height < h - 1:
            candidates.append(contour)
    if candidates:
        return max(candidates, key=cv2.contourArea)
    return None


def pad_center(frame):
    contour = pad_contour(frame)
    return center(contour) if contour is not None else None


def check_pad(frame, height, next_height):
    contour = pad_contour(frame)
    h, w = frame.shape[:2]
    fx, fy = focal_lengths(frame)
    # Сколько метров от вертикали через центр дрона до ближайшей границы?
    clearance = cv2.pointPolygonTest(contour, (w / 2, h / 2), True)
    fits = clearance * height / max(fx, fy) > cfg.FOOTPRINT_RADIUS + cfg.LAND_ERROR
    x, y, pw, ph = cv2.boundingRect(contour)
    horizontal = max(w / 2 - x, x + pw - w / 2) / (w / 2)
    vertical = max(h / 2 - y, y + ph - h / 2) / (h / 2)
    # При снижении изображение увеличивается. Сядем с текущей высоты,
    # если следующий шаг обрежет контур уже отцентрированной площадки.
    visible_lower = max(horizontal, vertical) * height / next_height < 0.90
    return fits, visible_lower


def run(flight, camera):
    land_on_target(flight, camera, pad_center, check_pad)

def test_color_marker(camera_index=0):
    # Подключаем обычную веб-камеру компьютера
    camera = cv2.VideoCapture(camera_index)

    if not camera.isOpened():
        print("Не удалось подключить камеру")
        return

    print("Камера подключена!")
    print("Нажми ESC для выхода.")

    try:
        while True:
            # Получаем кадр с веб-камеры
            ret, frame = camera.read()

            if not ret:
                print("Не удалось получить изображение")
                break

            # Создаём маску нужного цвета
            mask = color_mask(frame, cfg.PAD_COLOR)

            # Ищем цветной маркер твоей функцией
            contour = pad_contour(frame)

            # Копируем кадр для отображения
            result = frame.copy()

            if contour is not None:
                # Рисуем найденный контур
                cv2.drawContours(
                    result,
                    [contour],
                    -1,
                    (0, 255, 0),
                    2
                )

                # Находим центр маркера
                point = center(contour)

                if point is not None:
                    cx, cy = point
                    cx = int(round(cx))
                    cy = int(round(cy))
                    cv2.circle(
                        result,
                        (cx, cy),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    cv2.putText(
                        result,
                        f"Center: {cx}, {cy}",
                        (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                cv2.putText(
                    result,
                    "MARKER FOUND",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2
                )

            else:
                cv2.putText(
                    result,
                    "MARKER NOT FOUND",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

            # Показываем исходное изображение и маску
            cv2.imshow("Webcam", result)
            cv2.imshow("Color mask", mask)

            # ESC — выход
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        camera.release()
        cv2.destroyAllWindows()
        print("Камера отключена.")


if __name__ == "__main__":
    test_color_marker(0)
    camera_mission(cfg.DOWN_CAMERA, run)
