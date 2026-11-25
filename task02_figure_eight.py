"""№ 2. Два круга радиусом 1 м с общей касательной в точке старта."""
import math
from flight import fly_points
from settings import BOTTOM


def figure_eight():
    radius = 1.0
    middle_z = BOTTOM + radius
    yield 0, 0, middle_z
    for side in (1, -1):
        for i in range(1, 37):
            angle = 2 * math.pi * i / 36
            # Меняем знак только у X. На втором круге обход меняет направление.
            x = side * radius * (1 - math.cos(angle))
            z = middle_z + radius * math.sin(angle)
            yield x, 0, z


if __name__ == "__main__":
    fly_points(figure_eight())
