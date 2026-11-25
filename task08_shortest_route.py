"""№ 8. Полный перебор порядков посещения, максимум 5! = 120 вариантов."""
from itertools import permutations, pairwise
from math import dist
from flight import Flight, airborne, connection, check_point, number
from settings import HEIGHT


def shortest_route(start, points):
    def length(order):
        path = (start, *(points[i] for i in order), start)
        return sum(dist(a, b) for a, b in pairwise(path))

    order = min(permutations(range(len(points))), key=length)
    return order, length(order)


def main():
    count = int(input("Количество точек (3–5): "))
    if not 3 <= count <= 5:
        raise ValueError("Нужно от 3 до 5 точек.")
    print("Вводите координаты в системе LPS площадки, в метрах.")
    points = [(number(f"X точки {i + 1}: "), number(f"Y точки {i + 1}: "))
              for i in range(count)]
    with connection() as drone:
        flight = Flight(drone)
        x0, y0, _ = flight.origin
        if len(set(points + [(x0, y0)])) != count + 1:
            raise ValueError("Контрольные точки и старт должны быть различны.")
        order, length = shortest_route((x0, y0), points)
        labels = [str(i + 1) for i in order]
        print(" → ".join(["Старт", *labels, "Старт"]))
        print(f"Длина маршрута в плоскости XY: {length:.3f} м")
        route = [(0, 0, HEIGHT)]
        for i in order:
            x, y = points[i]
            route.append((x - x0, y - y0, HEIGHT))
        route.append((0, 0, HEIGHT))
        for point in route:
            check_point(*point)
        with airborne(flight):
            for point in route:
                flight.go(*point)


if __name__ == "__main__":
    main()
