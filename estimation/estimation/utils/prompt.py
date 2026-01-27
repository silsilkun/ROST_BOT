# prompt.py
# 역할:
# - "분류 정책"과 "출력 형식"만 정의
# - 어떤 규칙이든 여기서만 바뀌도록 설계
# - 코드(gemini_api / logic)는 절대 수정하지 않아도 되게 만드는 파일

class PromptConfig:
    def __init__(self):
        # [수정 포인트] 모델/토큰/온도는 운영 중 바뀔 수 있음
        self.default_model = "gemini-2.5-flash"
        self.default_timeout = 20.0
        self.default_temp = 0.0
        self.default_max_tokens = 512

        # [수정 포인트] 분류 라벨 정의 (정책)
        self.allowed_labels = ["plastic", "can", "paper", "box", "unknown"]

        # [수정 포인트] 라벨 → ID 매핑 (코드 계약)
        self.label_to_id = {
            "plastic": 0.0,
            "can": 1.0,
            "paper": 2.0,
            "box": 3.0,
            "unknown": -1.0,
        }

        # [권장] 정책 버전 (디버깅/재현성)
        self.prompt_version = "waste-policy-v1.0"

    def get_prompt(self, expected_count: int) -> str:
        """
        expected_count:
          - 반드시 이 길이만큼의 JSON array를 출력해야 함
        """

        return (
            "You are an expert waste classification AI for a robotic sorting system.\n"
            "\n"
            "Your task is to classify each detected object into exactly ONE of the "
            "following categories:\n"
            f"{', '.join(self.allowed_labels)}\n"
            "\n"
            "----------------------------------------\n"
            "OUTPUT FORMAT (ABSOLUTE REQUIREMENT)\n"
            "----------------------------------------\n"
            f"- Return ONLY a valid JSON array of length {expected_count}.\n"
            "- Each element must be ONE string label from the allowed categories.\n"
            "- Do NOT include explanations, comments, markdown, or code fences.\n"
            "- The order of elements MUST match the input object order.\n"
            "\n"
            "----------------------------------------\n"
            "CLASSIFICATION POLICY\n"
            "----------------------------------------\n"
            "- Use visual appearance, shape, material, and context cues.\n"
            "- Apply any color, bounding box, or heuristic rules ONLY if they are "
            "visually reliable in the given image.\n"
            "- If a clear container type is visible, choose one of: plastic / can / paper / box.\n"
            "- Use \"unknown\" ONLY if the object is not visible, ambiguous, or not a container.\n"
            "\n"
            "----------------------------------------\n"
            "IMPORTANT\n"
            "----------------------------------------\n"
            "- Be decisive. Avoid overusing \"unknown\".\n"
            "- Output MUST strictly follow the specified JSON array format.\n"
        )