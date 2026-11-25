"""№ 11. Карта в метрах, живая точка и след. Команд управления здесь нет."""
import csv
import math
import time
import tkinter as tk
from datetime import datetime
from queue import Empty, Queue
from threading import Event, Thread

from flight import connection, number


def main():
    print("Введите границы реальной площадки в координатах LPS, м.")
    xmin, xmax = number("X слева: "), number("X справа: ")
    ymin, ymax = number("Y снизу: "), number("Y сверху: ")
    if xmin >= xmax or ymin >= ymax:
        raise ValueError("Максимумы должны быть больше минимумов.")
    root = tk.Tk()
    root.title("Пионер — карта полёта")
    width, height, margin = 840, 610, 52
    canvas = tk.Canvas(root, width=width, height=height, bg="white")
    canvas.pack()
    status = tk.Label(root, text="Подключение к дрону…")
    status.pack()
    # Один метр по X и Y занимает одинаковое число пикселей.
    scale = min((width - 2*margin)/(xmax-xmin), (height - 2*margin)/(ymax-ymin))

    def screen(x, y):
        return margin + (x-xmin)*scale, height-margin - (y-ymin)*scale

    left, bottom = screen(xmin, ymin)
    right, top = screen(xmax, ymax)
    canvas.create_rectangle(left, top, right, bottom, outline="#55728a")
    step = 1.0
    while step * scale < 40:
        step *= 2
    while step * scale > 125:
        step /= 2
    for axis, low, high in (("X", xmin, xmax), ("Y", ymin, ymax)):
        for i in range(math.ceil(low/step), math.floor(high/step) + 1):
            value = i * step
            if axis == "X":
                x, _ = screen(value, ymin)
                canvas.create_line(x, top, x, bottom, fill="#dae4ea")
                canvas.create_text(x, bottom+14, text=f"{value:g}")
            else:
                _, y = screen(xmin, value)
                canvas.create_line(left, y, right, y, fill="#dae4ea")
                canvas.create_text(left-22, y, text=f"{value:g}")
    canvas.create_text(right, bottom+34, text="X, м")
    canvas.create_text(left-15, top-15, text="Y, м")
    dot = canvas.create_oval(0, 0, 0, 0, fill="#d24d30", state="hidden")
    samples = Queue()
    stop = Event()

    def receive():
        try:
            with connection() as drone:
                while not stop.is_set():
                    stamp = time.monotonic()
                    point = drone.get_local_position_lps()
                    samples.put((stamp, point))
                    stop.wait(0.15)
        except (Exception, SystemExit) as error:
            samples.put((time.monotonic(), str(error) or "Ошибка подключения"))

    filename = datetime.now().strftime("track_%Y%m%d_%H%M%S_%f.csv")
    with open(filename, "w", encoding="utf-8", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["seconds", "lps_x_m", "lps_y_m"])
        start = time.monotonic()
        last_seen = start
        previous = None
        error_text = "Нет свежей позиции LPS"

        def redraw():
            nonlocal last_seen, previous, error_text
            while True:
                try:
                    stamp, point = samples.get_nowait()
                except Empty:
                    break
                if isinstance(point, str):
                    error_text = point
                    continue
                if point is None or not all(math.isfinite(v) for v in point):
                    continue
                x, y, _ = point
                writer.writerow([round(stamp-start, 3), x, y])
                file.flush()
                if time.monotonic() - stamp > 2:
                    continue
                px, py = screen(x, y)
                if previous is not None and stamp-last_seen <= 2:
                    canvas.create_line(*previous, px, py, fill="#176787", width=2)
                previous = px, py
                last_seen = stamp
                canvas.coords(dot, px-5, py-5, px+5, py+5)
                canvas.itemconfigure(dot, state="normal")
                canvas.tag_raise(dot)
                status.config(text=f"X {x:.2f} м     Y {y:.2f} м     Сетка {step:g} м     {filename}")
            if time.monotonic() - last_seen > 2:
                canvas.itemconfigure(dot, state="hidden")
                status.config(text=error_text)
            root.after(80, redraw)

        worker = Thread(target=receive, daemon=True)
        worker.start()
        root.after(0, redraw)
        try:
            root.mainloop()
        finally:
            stop.set()
            worker.join(timeout=2)


if __name__ == "__main__":
    main()
