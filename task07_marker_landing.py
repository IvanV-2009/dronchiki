"""№ 7. Среди видимых маркеров выбираем только заданный ID."""
import settings as cfg
from vision import camera_mission, land_on_target, marker_reader, read_id


def main():
    wanted = read_id("ID посадочного маркера: ")
    markers = marker_reader()

    def target(frame):
        return markers(frame).get(wanted)

    def run(flight, camera):
        land_on_target(flight, camera, target)

    camera_mission(cfg.DOWN_CAMERA, run)


if __name__ == "__main__":
    main()
