# utils.py
# 파싱 도구 ㅎㅎ 잡일 담당 ㅎㅎ
import json
import re
import os

class ImageLoader:
    def read_file(self, path: str) -> bytes | None:
        return read_image_file(path)

    def guess_mime_type(self, filename_or_format: str) -> str:
        return guess_mime_type(filename_or_format)

class ResponseParser:
    def parse_labels(self, text: str) -> list[str]:
        return parse_labels_from_text(text)

    def convert_to_ids(self, labels: list[str], label_map: dict, unknown_id: float) -> list[float]:
        return convert_labels_to_ids(labels, label_map, unknown_id)

def guess_mime_type(filename_or_format: str) -> str:
    lower = (filename_or_format or "").lower()
    if "png" in lower:
        return "image/png"
    if "jpg" in lower or "jpeg" in lower:
        return "image/jpeg"
    return "application/octet-stream"  # Default

def guess_mime_from_format(fmt: str) -> str:
    return guess_mime_type(fmt)

def read_image_file(path: str) -> bytes | None:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "rb") as f:
            return f.read()
    except OSError:
        return None

def parse_labels_from_text(text: str) -> list[str]:
    # 코드 블록 제거 (```json ... ```)
    match = re.search(r"```(?:json)?\s*(.*?)```", text, re.DOTALL | re.IGNORECASE)
    cleaned = match.group(1) if match else text
    cleaned = cleaned.strip()

    parsed = None
    try:
        parsed = json.loads(cleaned)
    except json.JSONDecodeError:
        # 대괄호만 추출해서 재시도
        start, end = cleaned.find("["), cleaned.rfind("]")
        if start != -1 and end > start:
            try:
                parsed = json.loads(cleaned[start : end + 1])
            except json.JSONDecodeError:
                pass

    # 결과가 dict면 labels 키 추출, 아니면 리스트 자체
    if isinstance(parsed, dict):
        parsed = parsed.get("labels")

    labels = []
    if isinstance(parsed, list):
        for item in parsed:
            if isinstance(item, dict):
                label = item.get("label") or item.get("name") or item.get("type")
                if label is None:
                    continue
                labels.append(str(label))
            else:
                labels.append(str(item))
    else:
        # 최후의 수단: 콤마/줄바꿈으로 분리
        tokens = re.split(r"[,\n]+", cleaned)
        labels = [t.strip() for t in tokens if t.strip()]
    
    return labels

def convert_labels_to_ids(labels: list[str], label_map: dict, unknown_id: float) -> list[float]:
    ids = []
    for label in labels:
        norm = label.strip().lower()
        if norm == "unknown":
            ids.append(unknown_id)
            continue
        
        matched = next((k for k in label_map if k in norm), None)
        ids.append(label_map[matched] if matched else unknown_id)
    return ids
