"""Связь, границы и ожидание фактического прибытия. Координаты задач — от старта."""
import math
import time
from contextlib import contextmanager

from pioneer_sdk2 import Pioneer
import settings as cfg


def number(prompt):
    value = float(input(prompt).strip().replace(",", "."))
    if not math.isfinite(value):
        raise ValueError("Нужно конечное число.")
    return value


def position(drone):
    point = drone.get_local_position_lps()
    if point is None or not all(math.isfinite(v) for v in point):
        raise RuntimeError("Нет координат LPS.")
    return point


def check_point(x, y, z):
    for axis, value, (low, high) in zip("XYZ", (x, y, z),
                                        (cfg.X_LIMITS, cfg.Y_LIMITS, cfg.Z_LIMITS)):
        if not low <= value <= high:
            raise ValueError(f"{axis} = {value:.2f} м вне настроенной зоны [{low}, {high}].")


class Flight:
    def __init__(self, drone):
        self.drone = drone
        self.origin = position(drone)
        self.yaw = drone.get_local_yaw_lps()
        if self.yaw is None or not math.isfinite(self.yaw):
            raise RuntimeError("Нет курса LPS.")

    def here(self):
        return tuple(v - start for v, start in zip(position(self.drone), self.origin))

    def aim(self, x, y, z):
        """Отправить цель. Курс корпуса во всех заданиях сохраняется."""
        check_point(x, y, z)
        x0, y0, z0 = self.origin
        sent = self.drone.go_to_local_point(x0 + x, y0 + y, z0 + z, self.yaw)
        if sent is False:  # На успех SDK 0.16.1 может вернуть None.
            raise RuntimeError("Автопилот отклонил точку.")

    def go(self, x, y, z, tolerance=0.08):
        self.aim(x, y, z)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if math.dist(self.here(), (x, y, z)) <= tolerance:
                return
            time.sleep(0.08)
        raise TimeoutError(f"Дрон не достиг точки ({x:.2f}, {y:.2f}, {z:.2f}).")

    def hold(self, target, seconds, tolerance=0.12):
        """Весь заданный интервал должен пройти внутри допуска."""
        self.aim(*target)
        deadline = time.monotonic() + seconds + 12
        since = time.monotonic()
        while time.monotonic() < deadline:
            now = time.monotonic()
            if math.dist(self.here(), target) > tolerance:
                since = now
            elif now - since >= seconds:
                return
            time.sleep(0.08)
        raise RuntimeError("Не удалось удержать точку.")


@contextmanager
def connection():
    drone = Pioneer(tcp=cfg.TCP, logger=False)
    try:
        yield drone
    finally:
        drone.close_connection()


@contextmanager
def airborne(flight):
    """Все ветви выхода после запуска моторов запрашивают штатную посадку."""
    status = flight.drone.get_nav_status_lps()
    if getattr(status, "name", None) != "OK":
        raise RuntimeError("LPS не подтверждает надёжную позицию.")
    if not flight.drone.arm():
        raise RuntimeError("Автопилот не подтвердил запуск моторов.")
    try:
        if not flight.drone.takeoff():
            raise RuntimeError("Автопилот не подтвердил взлёт.")
        yield
    finally:
        if not flight.drone.land():
            raise RuntimeError("Посадка не подтверждена. Управление у оператора.")


def fly_points(points, tolerance=0.08):
    points = list(points)
    for point in points:
        check_point(*point)
    with connection() as drone:
        flight = Flight(drone)
        with airborne(flight):
            for point in points:
                flight.go(*point, tolerance=tolerance)
        # После подтверждённой посадки сравним с точкой взлёта по критерию XZ.
        x, _, z = flight.here()
        error = math.hypot(x, z)
        print(f"Отклонение от старта после посадки по XZ: {error:.3f} м")
        if error > 0.2:
            print("Допуск 0,2 м превышен.")
