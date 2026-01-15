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
            "### CRITICAL RULE (HIGHEST PRIORITY):\n"
            "- BLUE box = ALWAYS \"plastic\". (Transparent PET cups, etc.)\n"
            "- Even if it looks metallic, if it's in a BLUE box, it's \"plastic\".\n"
            "\n"
            "### DETAILED CLASSIFICATION CRITERIA:\n"
            "1. plastic: Objects in BLUE boxes OR clear/transparent PET bottles/containers.\n"
            "2. can: Look for metallic luster, circular rims, or a pull-tab on top. Usually has a reflective surface.\n"
            "3. paper: Food/Beverage cartons (Milk, Yogurt, Juice). Look for printed graphics, white/glossy paper texture, and folded 'ears' or gabled tops. Not brown.\n"
            "4. box: Shipping containers made of brown kraft paper or corrugated cardboard. Look for tape marks, fold lines, and a matte, fibrous texture.\n"
            "5. unknown: Non-container objects (e.g., robot parts, floor, tools).\n"
            "\n"
            "### DECISION LOGIC FOR AMBIGUITY:\n"
            "- If it is 'Brown & Corrugated' -> it is a \"box\".\n"
            "- If it is 'White/Colored & Printed' (like a yogurt pack) -> it is \"paper\".\n"
            "- If it has a 'Circular Metal Opening' -> it is a \"can\".\n"
            "\n"
            "### OUTPUT RULES:\n"
            f"- Array Length: {expected_count}\n"
            "- Order: Follow the input coordinate order strictly.\n"
            "- Be decisive: If it's a waste container, map it to one of the 4 categories.\n"
        )