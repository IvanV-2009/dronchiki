"""№ 1. Вертикальный круг в плоскости XZ."""
import math
from flight import number, fly_points
from settings import BOTTOM, Z_LIMITS


def circle(radius):
    steps = max(36, math.ceil(2 * math.pi * radius / 0.25))
    for i in range(steps + 1):
        angle = 2 * math.pi * i / steps
        x = radius * math.sin(angle)
        z = BOTTOM + radius * (1 - math.cos(angle))
        yield x, 0, z


if __name__ == "__main__":
    radius = number("Радиус круга, м: ")
    fly_points(circle(radius), tolerance=min(0.08, radius / 8))
