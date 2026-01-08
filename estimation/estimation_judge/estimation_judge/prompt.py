# prompt.py
# 프롬프트 수정만 조지면 됨 끼양

# 허용 라벨 및 ID 매핑
ALLOWED_LABELS = ["can", "plastic", "paper", "box"]
LABEL_TO_ID = {"can": 0.0, "plastic": 1.0, "paper": 2.0, "box": 3.0}

# 기본 설정값
DEFAULT_MODEL = "gemini-2.5-flash"
DEFAULT_TIMEOUT = 20.0
DEFAULT_TEMP = 0.0
DEFAULT_MAX_TOKENS = 1024

# 프롬프트 템플릿
def get_prompt(expected_count, allowed_labels):
    return (
        "너는 재활용 분류를 하는 분류기야. "
        f"허용 라벨: {', '.join(allowed_labels)}. "
        f"반드시 길이가 {expected_count}인 JSON 배열만 반환해. "
        "설명이나 코드블록 없이 JSON만 출력해. "
        "출력은 반드시 [로 시작하고 ]로 끝나야 해. "
        "출력 예시: [\"plastic\"]. "
        "순서는 인식 좌표 목록 순서와 반드시 일치해야 해. "
        "배경/그림자는 무시하고 물체 자체만 보고 판단해. "
        "인쇄문자, 로고, 포장재는 내용물이 아니라 물체의 일부로 간주해. "
        "잔여 액체/내용물은 무시하고 용기 재질/형태로 분류해. "
        "사진에서 제품명이 보이면 그 정보를 분류에 활용해. "
        "예외 규칙: 'Maeil' 혹은 '바이오'가 보이면 라벨은 paper. "
        "겉모습만으로 재질이 불확실하면 'unknown'을 반환해."
    )
