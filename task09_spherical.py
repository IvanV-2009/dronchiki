"""№ 9. Азимут отсчитываем от +X к +Y, угол возвышения — от горизонта."""
import math
from flight import Flight, airborne, check_point, connection, number
from settings import HEIGHT


def to_xyz(distance, azimuth, elevation):
    if distance <= 0 or not -90 <= elevation <= 90:
        raise ValueError("Проверь расстояние и угол возвышения (от −90 до 90°).")
    a, b = math.radians(azimuth), math.radians(elevation)
    horizontal = distance * math.cos(b)
    return horizontal * math.cos(a), horizontal * math.sin(a), distance * math.sin(b)


def main():
    target = to_xyz(number("Расстояние, м: "), number("Азимут, °: "),
                    number("Угол возвышения, °: "))
    print("От точки взлёта: X = %.3f, Y = %.3f, Z = %.3f м" % target)
    check_point(*target)
    check_point(0, 0, HEIGHT)
    with connection() as drone:
        flight = Flight(drone)
        print("Цель в LPS:", tuple(round(a + b, 3) for a, b in zip(flight.origin, target)))
        with airborne(flight):
            flight.go(*target)
            flight.hold(target, seconds=3.2)
            flight.go(0, 0, HEIGHT)


if __name__ == "__main__":
    main()
