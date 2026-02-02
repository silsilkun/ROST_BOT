from .gemini_api import GeminiClient
from .utils import parse_labels_from_text, convert_labels_to_ids


class EstimationLogic:
    def __init__(self, prompt_cfg, api_key, logger=None):
        self.cfg, self.log = prompt_cfg, logger
        self.client = GeminiClient(api_key, self.cfg.default_model, self.cfg.default_timeout,
                                   self.cfg.default_temp, self.cfg.default_max_tokens)
        self._pcnt, self._pp = None, None  # prompt cache

    def run_inference(self, img_bytes: bytes, mime_type: str, expected_cnt: int, unknown_id: float) -> list[float]:
        def warn(m): (self.log(m) if self.log else print(m))
        if expected_cnt <= 0: return []
        if not img_bytes: return [unknown_id] * expected_cnt
        mime_type = mime_type or "application/octet-stream"

        if self._pcnt != expected_cnt:
            self._pcnt, self._pp = expected_cnt, self.cfg.get_prompt(expected_cnt)

        retries = int(getattr(self.cfg, "max_retries", 1))  # [수정 포인트]
        for i in range(retries + 1):
            try:
                txt = self.client.generate(self._pp, img_bytes, mime_type)
                labels = parse_labels_from_text(txt)
                ids = convert_labels_to_ids(labels, self.cfg.label_to_id, unknown_id)
                out = [float(x) if x is not None else float(unknown_id) for x in (ids or [])]
                return (out + [float(unknown_id)] * expected_cnt)[:expected_cnt]
            except Exception as e:
                if i == retries:
                    warn(f"[Logic Error] inference failed -> unknown. err={e}")
                    return [float(unknown_id)] * expected_cnt