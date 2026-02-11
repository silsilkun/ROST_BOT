# R.O.S.T Perception → Control 인수인계 문서

**작성일:** 2025-02-11
**작성:** 게토스구루 (PM) + Claude (코드 작성 보조)
**대상:** 수환님 (Control/Node 통합 담당)

---

## 1. 현재 상태 요약

Perception 파트의 **기능 함수(utils)** 개발이 완료되었습니다.
ROS2 노드 없이 독립 실행 가능한 테스트 파이프라인(`test_pipeline.py`)으로 검증했습니다.

**검증 완료:**
- ✅ RealSense 카메라 RGB 촬영
- ✅ ROI 선택 (마우스 드래그)
- ✅ Bin 위치 7개 선택 (마우스 클릭)
- ✅ Gemini API 연결 및 Step 1~3 호출
- ✅ Qt 백엔드 OpenCV 호환 처리

**아직 미검증 / 남은 작업:**
- ⬜ RealSense Depth 스트림 동시 촬영 (`capture_snapshot_and_depth`)
- ⬜ Calibration 좌표 변환 (camcalib.npz 적용)
- ⬜ 전체 1사이클 테스트 (메뉴 6번)
- ⬜ ROS2 노드 연결
- ⬜ Output → Control 전달 통신 구조

---

## 2. 파일 구조

```
perception/
├── nodes/
│   └── (수환님이 작성할 ROS2 실행 노드)
└── utils/
    ├── config.py                  # 전체 설정 상수
    ├── setup_functions.py         # ROI + Bin 위치 선택 (Qt 호환)
    ├── camera_capture.py          # RealSense RGB + Depth 캡처
    ├── gemini_functions_v2.py     # Gemini API Step 1~3 (4개 개선사항 반영)
    ├── calibration.py             # 좌표 변환 (파트장님 coordinate.py 기반)
    ├── camcalib.npz               # 캘리브레이션 데이터 (1차 데모 값)
    ├── .env                       # API 키 (GEMINI_API_KEY=...)
    ├── main_pipeline.py           # ※ 참조용, 함수 연결 순서 설명
    └── test_pipeline.py           # 독립 테스트 (노드 없이 실행)
```

---

## 3. 7-Step 파이프라인 흐름

```
[초기화]
  init_camera() → (pipeline, align) 튜플
  init_gemini_client() → gemini 객체

[1회 설정]
  capture_snapshot(cam) → frame
  select_roi(frame) → roi (x, y, w, h)
  select_bin_positions(frame) → bins {"box": (bx,by), ...}

[반복 루프]
  ┌─ capture_snapshot_and_depth(cam) → frame, depth_m
  │  crop_to_roi(frame, roi) → roi_img
  │
  ├─ Step 1: check_objects_exist(gemini, roi_img) → True/False
  │  (False면 "분리수거 완료" → 루프 종료)
  │
  ├─ Step 2: select_target_object(gemini, roi_img) → target
  │  target = {"label", "bbox", "center", "angle"}
  │
  ├─ Step 3: classify_object(gemini, bbox_img) → type_id (0~6)
  │
  ├─ 좌표 변환: gemini_to_robot(center, roi, depth_m) → tx, ty, tz
  │
  └─ Output: [type_id, tx, ty, tz, t_angle, bx, by]
             → Control 파트로 전달
```

---

## 4. Output 형식

```python
output = [type_id, tx, ty, tz, t_angle, bx, by]
```

| 인덱스 | 이름 | 타입 | 단위 | 설명 |
|--------|------|------|------|------|
| 0 | type_id | int | - | 카테고리 (아래 표 참조) |
| 1 | tx | float | cm | 로봇 작업좌표 X |
| 2 | ty | float | cm | 로봇 작업좌표 Y |
| 3 | tz | float | cm | 로봇 작업좌표 Z (depth 기반) |
| 4 | t_angle | float | ° | 그리퍼 접근 각도 (0~180) |
| 5 | bx | int | px | 쓰레기통 위치 X (이미지 픽셀) |
| 6 | by | int | px | 쓰레기통 위치 Y (이미지 픽셀) |

### type_id 매핑

| type_id | category | 설명 |
|---------|----------|------|
| 0 | box | 박스/종이박스 |
| 1 | paper | 종이 |
| 2 | plastic | 플라스틱 |
| 3 | vinyl | 비닐 |
| 4 | glass | 유리 |
| 5 | can | 캔 |
| 6 | unknown | 미분류 |

---

## 5. 주요 결정사항 & 논의 필요 사항

### 5-1. Depth / 높이값 처리 (논의 필요!)

현재 calibration.py는 **RealSense depth**로 tx, ty, tz를 모두 계산합니다.
그런데 **투명한 플라스틱 등은 RealSense depth가 안 잡히는 문제**가 있어서,
높이(z)는 **ToF 센서에 일임**하자는 논의가 있었습니다.

**선택지:**
1. **고정 depth 상수** — 카메라~테이블 거리가 일정하니 상수로 tx, ty만 계산
2. **ToF 값을 calibration에 넘기기** — ToF depth → pixel_to_robot()에 전달
3. **RealSense depth 그대로 사용** — 투명 물체는 실패 가능성 있음

→ 파트장님과 상의 후 결정 필요

### 5-2. init_camera() 리턴값 변경

```python
# 이전 (RGB만)
pipeline = init_camera()

# 현재 (RGB + Depth)
cam = init_camera()  # (pipeline, align) 튜플
```

모든 함수에 `cam` 튜플을 넘겨야 합니다. `pipeline` 단독으로 넘기면 에러.

### 5-3. Bin 좌표 = 이미지 픽셀 좌표

현재 bin 위치는 **카메라 이미지상 픽셀 좌표**입니다.
로봇 좌표로 변환이 필요하면 calibration의 `pixel_to_robot()`를 사용하면 됩니다.

### 5-4. Gemini 파일명

`gemini_functions.py` → **`gemini_functions_v2.py`**로 이름이 바뀌었습니다.
import 시 주의해주세요.

---

## 6. 파일별 함수 목록 (빠른 참조)

### config.py
설정 상수만 있음. import해서 사용.

### camera_capture.py
| 함수 | 입력 | 출력 |
|------|------|------|
| `init_camera()` | - | `(pipeline, align)` |
| `stop_camera(cam)` | cam 튜플 | - |
| `capture_snapshot(cam)` | cam 튜플 | RGB ndarray |
| `capture_snapshot_and_depth(cam)` | cam 튜플 | `(color, depth_m)` |
| `crop_to_roi(frame, roi)` | 이미지, (x,y,w,h) | 크롭 이미지 |
| `crop_to_bbox(roi_img, bbox, margin)` | ROI이미지, [ymin,xmin,ymax,xmax] | 크롭 이미지 |

### gemini_functions_v2.py
| 함수 | 입력 | 출력 |
|------|------|------|
| `init_gemini_client()` | - | gemini 객체 |
| `check_objects_exist(gemini, image)` | gemini, ndarray | `True/False` |
| `select_target_object(gemini, image)` | gemini, ndarray | `{"label","bbox","center","angle"}` or None |
| `classify_object(gemini, image)` | gemini, ndarray | `type_id` (int 0~6) |

### calibration.py
| 함수 | 입력 | 출력 |
|------|------|------|
| `pixel_to_robot(u, v, depth_map_m)` | 픽셀좌표, depth맵 | `(tx, ty, tz)` cm or None |
| `gemini_to_pixel(center, roi)` | [cy,cx], (x,y,w,h) | `(u, v)` 픽셀 |
| `gemini_to_robot(center, roi, depth_m)` | [cy,cx], roi, depth맵 | `(tx, ty, tz)` cm or None |

### setup_functions.py
| 함수 | 입력 | 출력 |
|------|------|------|
| `select_roi(frame)` | RGB 이미지 | `(x, y, w, h)` or None |
| `select_bin_positions(frame)` | RGB 이미지 | `{"box":(bx,by), ...}` or None |
| `close_setup_window()` | - | - |

※ ROI와 Bin 선택은 **하나의 창(`_WIN`)** 을 공유합니다. 중간에 창을 닫으면 Qt 에러 발생.

---

## 7. 테스트 실행 방법

```bash
# 1. 환경 준비
cd perception/utils/
pip install google-genai pyrealsense2 python-dotenv opencv-python

# 2. .env 파일 확인
cat .env
# GEMINI_API_KEY=AIzaSy...

# 3. camcalib.npz 확인
ls camcalib.npz

# 4. 실행
python test_pipeline.py
```

메뉴 순서: `2` (초기 설정) → `1` (카메라 확인) → `6` (전체 1사이클)

---

## 8. 알려진 이슈

1. **Qt 백엔드** — OpenCV 창을 닫았다 다시 여는 게 불가능. 반드시 `cv2.startWindowThread()` 호출 필요.
2. **카메라 점유** — 비정상 종료 시 `Device or resource busy` 에러. `pkill -f test_pipeline` 후 재실행.
3. **Gemini API 비용** — 테스트 시 매 호출마다 API 사용. Step 1~3 각각 별도 호출.

---

*문의: 게토스구루 (PM) 또는 Claude 대화 기록 참조*
