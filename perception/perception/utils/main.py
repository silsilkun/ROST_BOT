# main.py
import time
import threading

from . import realsense_loop
from . import click_points
from . import pipeline


last_result = None
running = True


def printer_loop():
    """1초에 1번 last_result 출력"""
    global running
    while running:
        print("processed_result =", last_result)
        time.sleep(1.0)


def main():
    global last_result, running

    def on_save_and_store():
        nonlocal_last = pipeline.save_cam()
        # save_cam()이 None을 리턴해도 그대로 저장
        globals()["last_result"] = nonlocal_last['vis']
        return nonlocal_last['vis']

    # 출력 전용 스레드 시작
    t = threading.Thread(target=printer_loop, daemon=True)
    t.start()

    try:
        realsense_loop.run(
            on_save=on_save_and_store,
            on_reset=click_points.reset_points,
            on_click=click_points.mouse_callback,
            update_depth_frame=click_points.update_depth_frame,
            update_color_image=click_points.update_color_image,
            get_points=click_points.get_saved_points,
        )
    finally:
        running = False


if __name__ == "__main__":
    main()