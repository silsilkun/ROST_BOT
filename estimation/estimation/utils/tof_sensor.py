"""
R.O.S.T - ToF 센서 인터페이스 (tof_sensor.py)
아두이노 ToF 센서에서 depth(거리) 값을 시리얼로 읽어온다.
모든 물체의 높이를 이 센서 하나로 통일 측정.
"""

import time
from config import TOF_SERIAL_PORT, TOF_BAUD_RATE, TOF_TIMEOUT

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[경고] pyserial 없음 → pip install pyserial")


def init_tof_sensor():
    """ToF 센서 시리얼 연결. 실패/테스트 시 None 반환."""
    if not SERIAL_AVAILABLE:
        return None
    try:
        # [수정 포인트] 포트/보드레이트는 config.py에서 관리
        ser = serial.Serial(port=TOF_SERIAL_PORT,
                            baudrate=TOF_BAUD_RATE,
                            timeout=TOF_TIMEOUT)
        time.sleep(2)  # 아두이노 리셋 대기
        ser.flushInput()
        print(f"[ToF] 연결 완료 ({TOF_SERIAL_PORT})")
        return ser
    except serial.SerialException as e:
        print(f"[에러] ToF 연결 실패: {e}")
        return None


def close_tof_sensor(ser):
    """시리얼 연결 종료"""
    if ser is not None and ser.is_open:
        ser.close()


def read_depth(ser) -> float:
    """
    ToF 센서에서 거리값 1회 읽기 (mm 단위).
    ser=None이면 테스트용 더미값 반환.
    실패 시 -1.0 반환.

    [전제] 아두이노가 Serial.println(distance_mm) 형태로 출력.
    """
    if ser is None:
        return 250.0  # 테스트용 더미값

    try:
        ser.flushInput()
        # [수정 포인트] 아두이노에 측정 명령이 필요하면 아래 주석 해제
        # ser.write(b'M')
        line = ser.readline().decode('utf-8').strip()
        if not line:
            print("[경고] ToF 응답 없음")
            return -1.0
        return float(line)
    except (ValueError, UnicodeDecodeError, serial.SerialException) as e:
        print(f"[에러] ToF 읽기 실패: {e}")
        return -1.0


def read_depth_stable(ser, n_samples: int = 5, delay: float = 0.05) -> float:
    """
    여러 번 측정 → 평균값 반환 (노이즈 제거용).
    유효한 값이 하나도 없으면 -1.0 반환.

    [수정 포인트] 측정 횟수/간격을 바꾸려면 파라미터만 수정
    """
    readings = []
    for _ in range(n_samples):
        val = read_depth(ser)
        if val > 0:
            readings.append(val)
        time.sleep(delay)

    if not readings:
        print("[경고] 유효한 ToF 측정값 없음")
        return -1.0

    avg = sum(readings) / len(readings)
    print(f"[ToF] {avg:.1f}mm (유효 {len(readings)}/{n_samples})")
    return avg
