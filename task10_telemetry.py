"""№ 10. Чтение по Enter. Полётом по-прежнему управляет оператор с пульта."""
from datetime import datetime
from flight import connection, number


def show(drone, empty_voltage, full_voltage):
    xyz = drone.get_local_position_lps()
    altitude = drone.get_altitude()
    battery = drone.get_battery_status()
    angles = drone.get_orientation()
    print("\nЗамер", datetime.now().strftime("%H:%M:%S"))
    print("Позиция:", "нет данных" if xyz is None else "X %.2f, Y %.2f, Z %.2f м" % xyz)
    print("Высота:", "нет данных" if altitude is None else f"{altitude:.2f} м")
    if battery is None or battery[0] is None:
        print("Аккумулятор: нет данных")
    else:
        voltage = battery[0]
        fraction = (voltage - empty_voltage) / (full_voltage - empty_voltage)
        percent = 100 * max(0, min(1, fraction))
        print(f"Аккумулятор: {voltage:.2f} В, грубая оценка по напряжению ≈ {percent:.0f}%")
    print("Ориентация:", "нет данных" if angles is None
          else "крен %.2f°, тангаж %.2f°, рыскание %.2f°" % angles)


def main():
    print("Введите напряжения всего пакета по характеристике вашего аккумулятора.")
    empty = number("Нижнее рабочее напряжение, В: ")
    full = number("Полностью заряженный аккумулятор, В: ")
    if not 0 < empty < full:
        raise ValueError("Требуется 0 < нижнее напряжение < полное напряжение.")
    with connection() as drone:
        while input("Enter — новый замер, q — выход: ").strip().lower() != "q":
            show(drone, empty, full)


if __name__ == "__main__":
    main()
