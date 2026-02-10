"""
R.O.S.T - 초기 설정 함수 (setup_functions.py)
ROI 선택(마우스 드래그) + bin 위치 7개 선택(마우스 클릭)
프로그램 시작 시 1회만 실행한다.
"""

import cv2
import numpy as np
from config import CATEGORIES


def select_roi(frame: np.ndarray) -> tuple:
    """
    마우스 드래그로 ROI(관심 영역)를 선택한다.
    → 이 네모 안의 영역만 Gemini한테 보낸다.
    Returns: (x, y, w, h) 또는 실패 시 None

    [변경] cv2.selectROI 대신 마우스 콜백 직접 구현 (Qt 백엔드 호환)
    """
    if frame is None or frame.size == 0:
        print("[에러] 프레임이 비어있습니다.")
        return None

    # 드래그 상태 저장용
    state = {"drawing": False, "start": None, "end": None, "done": False}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["drawing"] = True
            state["start"] = (x, y)
            state["end"] = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and state["drawing"]:
            state["end"] = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            state["drawing"] = False
            state["end"] = (x, y)
            state["done"] = True

    win = "Select ROI (drag then ENTER, ESC=cancel)"
    # Qt 백엔드 우회: imshow로 이미지를 먼저 띄워서 창을 강제 생성
    cv2.imshow(win, frame)
    cv2.waitKey(100)  # 100ms 대기 → Qt가 창 만들 시간 확보
    cv2.setMouseCallback(win, on_mouse)

    print("[설정] ROI를 마우스로 드래그 → ENTER 확정, ESC 취소")

    while True:
        display = frame.copy()
        # 드래그 중이거나 완료 시 사각형 표시
        if state["start"] and state["end"]:
            cv2.rectangle(display, state["start"], state["end"], (0, 255, 0), 2)
        cv2.imshow(win, display)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            cv2.destroyWindow(win)
            print("[경고] ROI 선택 취소됨")
            return None
        if key in (13, 32) and state["done"]:  # ENTER or SPACE
            break

    cv2.destroyWindow(win)

    # 좌표 정리 (드래그 방향 상관없이 정상화)
    sx, sy = state["start"]
    ex, ey = state["end"]
    x = min(sx, ex)
    y = min(sy, ey)
    w = abs(ex - sx)
    h = abs(ey - sy)

    if w == 0 or h == 0:
        print("[경고] ROI가 선택되지 않았습니다.")
        return None

    print(f"[설정] ROI 확정: x={x}, y={y}, w={w}, h={h}")
    return (x, y, w, h)


def select_bin_positions(frame: np.ndarray) -> dict:
    """
    7개 쓰레기통(bin) 위치를 마우스 클릭으로 지정한다.
    Returns: {"box": (bx,by), "paper": (bx,by), ...} 또는 실패 시 None
    """
    if frame is None or frame.size == 0:
        print("[에러] 프레임이 비어있습니다.")
        return None

    bin_positions = {}
    click_point = [None]  # 콜백에서 값을 넣기 위한 리스트

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point[0] = (x, y)

    # [수정 포인트] 카테고리 순서를 바꾸고 싶으면 여기만 수정
    categories_ordered = list(CATEGORIES.keys())

    print("\n[설정] 7개 bin 위치를 순서대로 클릭하세요.")
    win = "Bin 위치 선택"
    cv2.imshow(win, frame)
    cv2.waitKey(100)

    for category in categories_ordered:
        display = frame.copy()

        # 이미 선택된 위치들 표시 (초록 원)
        for prev_cat, (bx, by) in bin_positions.items():
            cv2.circle(display, (bx, by), 10, (0, 255, 0), -1)
            cv2.putText(display, prev_cat, (bx+15, by+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 안내 텍스트
        guide = f"{category} (type_id={CATEGORIES[category]}) 위치를 클릭"
        cv2.putText(display, guide, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow(win, display)
        cv2.setMouseCallback(win, on_click)

        click_point[0] = None
        print(f"  → {category} 위치를 클릭하세요...")

        while click_point[0] is None:
            key = cv2.waitKey(50)
            if key == 27:  # ESC → 취소
                cv2.destroyWindow(win)
                print("[경고] 취소됨")
                return None

        bx, by = click_point[0]
        bin_positions[category] = (bx, by)
        print(f"    ✓ {category}: ({bx}, {by})")

    cv2.destroyWindow(win)

    # [안전장치] 7개 전부 선택되었는지 확인
    assert len(bin_positions) == len(CATEGORIES), \
        f"bin 위치 {len(bin_positions)}개만 선택됨 (필요: {len(CATEGORIES)}개)"

    print("[설정] Bin 위치 선택 완료!")
    return bin_positions
