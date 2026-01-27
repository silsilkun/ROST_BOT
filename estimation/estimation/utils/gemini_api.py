# gemini_api.py (더 짧게: 빈 응답 방어 + 제한적 재시도만 유지)
import time
from google import genai
from google.genai import types


class GeminiClient:
    def __init__(self, api_key, model_name, timeout, temp, max_tokens, retries=1, backoff=0.5):
        self.c = genai.Client(api_key=api_key) if api_key else genai.Client()
        self.m = model_name
        self.t = float(temp)
        self.k = int(max_tokens)
        self.r = max(0, int(retries))
        self.b = float(backoff)

    def generate(self, prompt: str, image_bytes: bytes, mime_type: str) -> str:
        part = types.Part.from_bytes(data=image_bytes, mime_type=mime_type)
        cfg = types.GenerateContentConfig(temperature=self.t, max_output_tokens=self.k)
        err = None
        for i in range(self.r + 1):
            try:
                text = (self.c.models.generate_content(self.m, [part, prompt], config=cfg).text or "").strip()
                if not text:
                    raise RuntimeError("empty text")
                return text
            except Exception as e:
                err = e
                if i < self.r:
                    time.sleep(self.b * (2 ** i))
        raise RuntimeError(f"Gemini API failed: {err}")

    def close(self):
        try:
            self.c.close()
        except Exception:
            pass
