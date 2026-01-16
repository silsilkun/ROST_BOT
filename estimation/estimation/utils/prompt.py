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
            "You are the visual reasoning engine of a recycling sorting robot.\n"
            f"Analyze the materials of {expected_count} objects in the input image and output a JSON array.\n"
            "\n"

            "### [2. Few-shot Examples (learn exactly from these)]\n"

            "### [2-A. Target-oriented Prompt Set for PLASTIC (learn exactly from these)]"
            "Input: (a top-down image showing a transparent disposable plastic cup) -> Output: 'plastic'"
            "Input: (a side view of a clear disposable plastic drinking cup) -> Output: 'plastic'"
            "Input: (a transparent plastic cup used for takeaway beverages) -> Output: 'plastic'"
            "Input: (an empty transparent plastic cup placed on a flat surface) -> Output: 'plastic'"
            "Input: (a lightweight clear plastic cup for cold drinks) -> Output: 'plastic'"
            "Input: (a transparent plastic cup commonly used in cafes) -> Output: 'plastic'"
            "Input: (a single-use plastic drinking cup without a lid) -> Output: 'plastic'"
            "Input: (a plastic cup lying on its side on a dark background) -> Output: 'plastic'"
            "Input: (a clear plastic cup viewed from above) -> Output: 'plastic'"
            "Input: (a disposable transparent plastic cup with sharp reflections) -> Output: 'plastic'"

            "### [2-B. Target-oriented Prompt Set for CAN (learn exactly from these)]"
            "Input: (a top-down view of a metallic beverage can with a pull-tab opening) -> Output: 'can'"
            "Input: (a side view of a cylindrical aluminum can lying on its side) -> Output: 'can'"
            "Input: (a shiny metallic can with printed branding on the surface) -> Output: 'can'"
            "Input: (an empty aluminum drink can placed on a flat surface) -> Output: 'can'"
            "Input: (a silver metallic can reflecting light with a circular opening on top) -> Output: 'can'"
            "Input: (a beverage can viewed from above showing the pull-tab lid) -> Output: 'can'"
            "Input: (a lightweight aluminum can used for carbonated drinks) -> Output: 'can'"
            "Input: (a cylindrical metal can with glossy reflections) -> Output: 'can'"
            "Input: (a discarded aluminum drink can on a dark background) -> Output: 'can'"
            "Input: (a single aluminum beverage can with a pull-tab opening) -> Output: 'can'"

            "### [2-C. Target-oriented Prompt Set for PAPER (learn exactly from these)]"
            "Input: (a crumpled white paper object placed on a flat surface) -> Output: 'paper'"
            "Input: (a top-down view of a wrinkled paper waste ball) -> Output: 'paper'"
            "Input: (a crushed paper material with matte texture and no reflections) -> Output: 'paper'"
            "Input: (a piece of crumpled paper lying on a dark background) -> Output: 'paper'"
            "Input: (a lightweight paper waste object with irregular folded shape) -> Output: 'paper'"
            "Input: (a wrinkled paper ball commonly found in waste sorting) -> Output: 'paper'"
            "Input: (a non-glossy paper object compressed into a small ball) -> Output: 'paper'"
            "Input: (a discarded paper material with soft folds and creases) -> Output: 'paper'"
            "Input: (a crumpled sheet of paper without metallic or plastic shine) -> Output: 'paper'"
            "Input: (a paper waste item showing folded layers and matte surface) -> Output: 'paper'"

            "### [2-D. Target-oriented Prompt Set for BOX (learn exactly from these)]"
            "Input: (a brown cardboard box with a rigid cuboid shape) -> Output: 'box'"
            "Input: (a top-down view of a brown paper box made of cardboard material) -> Output: 'box'"
            "Input: (a small rectangular cardboard box placed on a flat surface) -> Output: 'box'"
            "Input: (a rigid paper box with folded edges and sharp corners) -> Output: 'box'"
            "Input: (a brown corrugated cardboard box used for packaging) -> Output: 'box'"
            "Input: (a lightweight cardboard box with a matte paper texture) -> Output: 'box'"
            "Input: (a paper-based box showing folded flaps and creases) -> Output: 'box'"
            "Input: (a rectangular brown box made of thick paper material) -> Output: 'box'"
            "Input: (a cardboard packaging box viewed from an angled top view) -> Output: 'box'"
            "Input: (a discarded brown cardboard box on a dark background) -> Output: 'box'"

            "### [2-E. Target-oriented Prompt Set for PAPER (Yogurt Carton, learn exactly from these)]"
            "Input: (a small yogurt paper carton with a blue plastic cap on top) -> Output: 'paper'"
            "Input: (a rectangular yogurt drink carton made of paper material) -> Output: 'paper'"
            "Input: (a paper-based yogurt container with printed branding) -> Output: 'paper'"
            "Input: (a top-down view of a yogurt carton with a screw cap) -> Output: 'paper'"
            "Input: (a white paper yogurt pack commonly used for liquid dairy products) -> Output: 'paper'"
            "Input: (a lightweight paper carton for yogurt drinks placed on a flat surface) -> Output: 'paper'"
            "Input: (a paper liquid carton with a blue cap and brand logo) -> Output: 'paper'"
            "Input: (a yogurt paper pack showing folded edges and printed labels) -> Output: 'paper'"
            "Input: (a disposable paper yogurt container used for beverage-style yogurt) -> Output: 'paper'"
            "Input: (a small rectangular paper carton for yogurt with a plastic lid) -> Output: 'paper'"
            "\n"

            f"### [3. Output Constraints]\n"
            f"- Return exactly {expected_count} elements in the form ['label1', 'label2', ...].\n"
            "- Do not use markdown fences (```).\n"
            "- If uncertain, choose the most likely category instead of 'unknown'."
        )