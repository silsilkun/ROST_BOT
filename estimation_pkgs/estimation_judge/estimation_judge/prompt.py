# prompt.py
# 프롬프트 수정만 조지면 됨 끼양

class PromptConfig:
    def __init__(self):
        # 기본 설정값
        self.default_model = "gemini-2.5-flash"
        self.default_timeout = 20.0
        self.default_temp = 0.0
        self.default_max_tokens = 1024
        
        # 라벨 데이터 (나중에 DB 연동 등 확장 고려)
        self.allowed_labels = ["plastic", "can", "paper", "box"]
        self.label_to_id = {"plastic": 0.0, "can": 1.0, "paper": 2.0, "box": 3.0}

    def get_prompt(self, expected_count: int) -> str:
        """
        현재 설정된 라벨을 기반으로 프롬프트를 동적으로 생성합니다.
        """
        labels_str = ', '.join(self.allowed_labels)
        return (
            "너는 재활용 분류를 하는 분류기야. "
            f"허용 라벨: {labels_str}. "
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
