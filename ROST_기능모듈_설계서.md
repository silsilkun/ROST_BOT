# R.O.S.T 쓰레기 분류 시스템 — 기능 모듈 설계서

## 전체 플로우 요약

```
[1회 설정] ROI 선택 + 7개 bin 위치 선택
     ↓
[루프 시작] ──────────────────────────────────┐
     ↓                                        │
  RGB 스냅샷 캡처 (ROI 크롭)                    │
     ↓                                        │
  Gemini: "ROI 안에 쓰레기 있어?" ──→ 없으면 종료  │
     ↓ 있으면                                  │
  Gemini: 가장 집기 쉬운 객체 1개 선택            │
         → bbox + center point(uv) + angle     │
     ↓                                        │
  Gemini: bbox 크롭 이미지로 카테고리 분류         │
         → type_id (7종 중 1개)                 │
     ↓                                        │
  ToF 센서: depth(tz) 측정                      │
     ↓                                        │
  Calibration: uv → 로봇 좌표 (tx, ty) 변환     │
     ↓                                        │
  Output 조립: [type_id, tx, ty, tz, t_angle,   │
                bx, by] → control 파트로 전달    │
     ↓                                        │
  control에서 해당 객체 분리수거                   │
     ↓                                        │
  ←──────────────────────────────────────────┘
```

## Gemini 호출 전략: 3단계 분리

정확도를 위해 Gemini API를 **3번 나눠서 호출**한다.

| 단계 | 목적 | 입력 | 출력 |
|------|------|------|------|
| Step 1 | 객체 존재 확인 | ROI 크롭 이미지 | `"yes"` / `"no"` |
| Step 2 | 타겟 선정 + 위치/각도 | ROI 크롭 이미지 | bbox, center point(uv), angle |
| Step 3 | 카테고리 분류 | bbox 크롭 이미지 (확대) | type_id (7종) |

**왜 나누는가?**
- Step 2는 "전체 장면"을 봐야 어떤 게 집기 쉬운지 판단 가능 (공간 추론)
- Step 3는 "그 물체만 확대"해서 봐야 분류 정확도가 올라감
- 문서 권장사항에도 "복잡한 문제는 작은 단계로 나눠라"고 명시됨

## 모듈 구성

| 파일 | 역할 |
|------|------|
| `config.py` | 상수, 카테고리 매핑, API 키 |
| `setup_functions.py` | ROI 선택, bin 위치 선택 (1회성) |
| `camera_capture.py` | RealSense RGB 캡처 + ROI 크롭 |
| `gemini_functions.py` | Gemini API 호출 3종 (핵심) |
| `tof_sensor.py` | ToF 아두이노 센서 depth 읽기 |
| `calibration.py` | uv → 로봇 좌표 변환 (placeholder) |

## 최종 Output 형식

```python
output = [type_id, tx, ty, tz, t_angle, bx, by]
```

| 인덱스 | 변수 | 설명 | 출처 |
|--------|------|------|------|
| 0 | type_id | 분류 카테고리 (0~6) | Gemini Step 3 |
| 1 | tx | 로봇 X 좌표 | Calibration(uv→로봇) |
| 2 | ty | 로봇 Y 좌표 | Calibration(uv→로봇) |
| 3 | tz | 높이 (depth) | ToF 센서 |
| 4 | t_angle | 그리퍼 각도 | Gemini Step 2 |
| 5 | bx | 해당 카테고리 bin의 X좌표 | 초기 설정 (2-2) |
| 6 | by | 해당 카테고리 bin의 Y좌표 | 초기 설정 (2-2) |

## 카테고리 매핑

| type_id | 카테고리 | 한국어 |
|---------|----------|--------|
| 0 | box | 박스/종이박스 |
| 1 | paper | 종이 |
| 2 | plastic | 플라스틱 |
| 3 | vinyl | 비닐 |
| 4 | glass | 유리 |
| 5 | can | 캔 |
| 6 | unknown | 미분류 |

## 파트장님 복귀 후 액션아이템 (Calibration)

1. 기존 캘리브레이션 코드 + 변환 행렬 공유받기
2. 변환 행렬이 현재 카메라 위치/각도에 유효한지 확인
3. 유효하지 않으면 재캘리브레이션 필요 → ArUco 마커 재배치 + 포인트 매핑
4. `calibration.py`의 placeholder 함수에 실제 변환 로직 채워넣기
5. uv 좌표 입력 → 로봇 좌표 출력 검증 테스트
