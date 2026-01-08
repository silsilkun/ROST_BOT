# logic.py
from .gemini_api import GeminiClient
from .utils import ResponseParser

class EstimationLogic:
    def __init__(self, prompt_cfg, api_key):
        self.cfg = prompt_cfg
        self.parser = ResponseParser()
        self.client = GeminiClient(
            api_key=api_key,
            model_name=self.cfg.default_model,
            timeout=self.cfg.default_timeout,
            temp=self.cfg.default_temp,
            max_tokens=self.cfg.default_max_tokens
        )

    def run_inference(self, img_bytes: bytes, mime_type: str, expected_cnt: int, unknown_id: float) -> list[float]:
        """이미지를 받아 추론 후 ID 리스트 반환 (로직의 핵심)."""
        try:
            prompt = self.cfg.get_prompt(expected_cnt)
            resp_text = self.client.generate(prompt, img_bytes, mime_type)
        except Exception as e:
            print(f"[Logic Error] Gemini API failed: {e}")  # 로거 대신 print 사용 (심플)
            return [unknown_id] * expected_cnt

        labels = self.parser.parse_labels(resp_text)
        ids = self.parser.convert_to_ids(labels, self.cfg.label_to_id, unknown_id)
        
        # 개수 맞추기 로직
        if len(ids) < expected_cnt:
            ids.extend([unknown_id] * (expected_cnt - len(ids)))
        return ids[:expected_cnt]
