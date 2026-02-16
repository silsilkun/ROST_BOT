"""
R.O.S.T - 초기 설정 함수 (setup_functions.py)
ROI 선택(마우스 드래그) + bin 위치 7개 선택(마우스 클릭)
프로그램 시작 시 1회만 실행한다.

[Qt 호환] 창을 닫았다 여는 대신, 하나의 창을 처음부터 끝까지 유지한다.
"""

import cv2
import numpy as np
from config import CATEGORIES


# ── 공용 창 이름 (프로그램 전체에서 1개만 사용) ────────
_WIN = "R.O.S.T Setup"


def _ensure_window(frame: np.ndarray):
    """
    창이 없으면 만들고, 있으면 그대로 사용.
    Qt 백엔드에서 창을 닫았다 여는 게 불가능하므로,
    프로그램 시작 시 1번만 호출하고 끝까지 유지한다.
    """
    cv2.imshow(_WIN, frame)
    cv2.waitKey(100)


def select_roi(frame: np.ndarray) -> tuple:
    """
    마우스 드래그로 ROI(관심 영역)를 선택한다.
    → 이 네모 안의 영역만 Gemini한테 보낸다.
    Returns: (x, y, w, h) 또는 실패 시 None
    """
    if frame is None or frame.size == 0:
        print("[에러] 프레임이 비어있습니다.")
        return None

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

    # 창 띄우기 + 마우스 콜백 등록
    _ensure_window(frame)
    cv2.setMouseCallback(_WIN, on_mouse)

    print("[설정] ROI를 마우스로 드래그 → ENTER 확정, ESC 취소")

    while True:
        display = frame.copy()
        if state["start"] and state["end"]:
            cv2.rectangle(display, state["start"], state["end"], (0, 255, 0), 2)
        cv2.imshow(_WIN, display)

        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC
            print("[경고] ROI 선택 취소됨")
            return None
        if key in (13, 32) and state["done"]:  # ENTER or SPACE
            break

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

    # ROI 결과 표시 (같은 창에서)
    result_display = frame.copy()
    cv2.rectangle(result_display, (x, y), (x+w, y+h), (0, 255, 0), 2)
    cv2.putText(result_display, f"ROI: ({x},{y}) {w}x{h} - Press any key",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow(_WIN, result_display)
    print(f"[설정] ROI 확정: x={x}, y={y}, w={w}, h={h}")
    print("  → 확인 후 아무 키나 누르세요")
    cv2.waitKey(0)

    return (x, y, w, h)


def select_bin_positions(frame: np.ndarray) -> dict:
    """
    7개 쓰레기통(bin) 위치를 마우스 클릭으로 지정한다.
    Returns: {"box": (bx,by), "paper": (bx,by), ...} 또는 실패 시 None

    [중요] select_roi와 같은 창(_WIN)을 공유한다. 창을 새로 만들지 않는다.
    """
    if frame is None or frame.size == 0:
        print("[에러] 프레임이 비어있습니다.")
        return None

    bin_positions = {}
    click_point = [None]

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point[0] = (x, y)

    # [수정 포인트] 카테고리 순서를 바꾸고 싶으면 여기만 수정
    categories_ordered = list(CATEGORIES.keys())

    # 같은 창에 콜백만 교체 (창을 새로 만들지 않음!)
    cv2.setMouseCallback(_WIN, on_click)

    print("\n[설정] 7개 bin 위치를 순서대로 클릭하세요.")

    for category in categories_ordered:
        display = frame.copy()

        # 이미 선택된 위치들 표시 (초록 원)
        for prev_cat, (bx, by) in bin_positions.items():
            cv2.circle(display, (bx, by), 10, (0, 255, 0), -1)
            cv2.putText(display, prev_cat, (bx+15, by+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 안내 텍스트
        guide = f"Click: {category} (type_id={CATEGORIES[category]})"
        cv2.putText(display, guide, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow(_WIN, display)

        click_point[0] = None
        print(f"  → {category} 위치를 클릭하세요...")

        while click_point[0] is None:
            key = cv2.waitKey(50)
            if key == 27:  # ESC → 취소
                print("[경고] 취소됨")
                return None

        bx, by = click_point[0]
        bin_positions[category] = (bx, by)
        print(f"    ✓ {category}: ({bx}, {by})")

    # [안전장치] 7개 전부 선택되었는지 확인
    assert len(bin_positions) == len(CATEGORIES), \
        f"bin 위치 {len(bin_positions)}개만 선택됨 (필요: {len(CATEGORIES)}개)"

    print("[설정] Bin 위치 선택 완료!")
    return bin_positions


def close_setup_window():
    """설정 완료 후 창 닫기. 이 함수만 창을 닫는다."""
    try:
        cv2.destroyWindow(_WIN)
        cv2.waitKey(100)
    except Exception:
        pass
