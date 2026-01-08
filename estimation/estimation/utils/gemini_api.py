# gemini_api.py
# 통신 모듈
import json
import urllib.request
import urllib.error
import base64

class GeminiClient:
    def __init__(self, api_key, model_name, timeout, temp, max_tokens):
        self.api_key = api_key
        self.model_name = model_name
        self.timeout = timeout
        self.temp = temp
        self.max_tokens = max_tokens

    def generate(self, prompt: str, image_bytes: bytes, mime_type: str) -> str:
        if not self.api_key:
            raise ValueError("API Key is missing")

        encoded = base64.b64encode(image_bytes).decode("ascii")
        payload = {
            "contents": [{
                "role": "user",
                "parts": [
                    {"text": prompt},
                    {"inlineData": {"mimeType": mime_type, "data": encoded}},
                ]
            }],
            "generationConfig": {
                "temperature": self.temp,
                "maxOutputTokens": self.max_tokens,
                "responseMimeType": "application/json",
            },
        }

        url = (
            "https://generativelanguage.googleapis.com/v1beta/models/"
            f"{self.model_name}:generateContent?key={self.api_key}"
        )
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )

        with urllib.request.urlopen(req, timeout=self.timeout) as resp:
            body = resp.read().decode("utf-8")

        data = json.loads(body)
        try:
            # 안전하게 텍스트 추출
            parts = data["candidates"][0]["content"]["parts"]
            return "\n".join([p.get("text", "") for p in parts if "text" in p])
        except (KeyError, IndexError):
            raise RuntimeError(f"Invalid Gemini response: {body}")