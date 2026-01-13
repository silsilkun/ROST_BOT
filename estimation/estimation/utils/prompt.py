# prompt.py
# 프롬프트 수정만 조지면 됨 끼양
# PromptConfig (Relaxed Unknown): unknown 최소화, 컨테이너면 반드시 0~3 중 선택

class PromptConfig:
    def __init__(self):
        self.default_model = "gemini-2.5-flash" 
        self.default_timeout = 20.0
        self.default_temp = 0.0
        self.default_max_tokens = 1024
        self.allowed_labels = ["plastic", "can", "paper", "box", "unknown"]
        self.label_to_id = {
            "plastic": 0.0, "can": 1.0, "paper": 2.0, "box": 3.0, "unknown": -1.0
        }

    def get_prompt(self, expected_count: int) -> str:
        return (
            "You are an expert waste classification AI for a robotic sorting system.\n"
            f"Return ONLY a JSON array of length {expected_count}. No talk, no markdown, just the array.\n"
            "\n"
            "### 🚨 CRITICAL RULE (HIGHEST PRIORITY):\n"
            "- IF an object is enclosed in a **BLUE bounding box**, it is ALWAYS classified as \"plastic\".\n"
            "- Ignore metallic reflections or gray colors if the box is BLUE. The blue box is the definitive indicator for a transparent plastic cup.\n"
            "\n"
            "### GENERAL CLASSIFICATION CRITERIA:\n"
            "- plastic: Plastic bottles, transparent PET cups, or objects in BLUE boxes.\n"
            "- can: Metallic beverage cans. Look for pull-tabs and solid, non-transparent surfaces.\n"
            "- paper: Paper-based packs (yogurt, milk).\n"
            "- box: Cardboard boxes.\n"
            "- unknown: Only if the object is not a container or invisible.\n"
            "\n"
            "### OUTPUT RULES:\n"
            f"- Length: {expected_count}\n"
            "- Order: Must match the input coordinate order.\n"
            "- Be decisive. If it's a container, choose one of the four main categories.\n"
        )