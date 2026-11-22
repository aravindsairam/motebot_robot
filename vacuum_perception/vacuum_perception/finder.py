"""The object finder: GroundingDINO (an open-vocabulary detector: it finds things named in words)
through Hugging Face transformers. No ROS imports. The model is loaded on first use."""

import threading
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

MODEL = "IDEA-Research/grounding-dino-tiny"  # Apache-2.0, from Hugging Face
REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"  # the version checked (its weights' sha256 matched the hub's)
BOX_THRESHOLD = 0.30
TEXT_THRESHOLD = 0.25


@dataclass
class Found:
    label: str
    score: float
    box: tuple  # x0, y0, x1, y1 in pixels


def prompt(names: Sequence[str]) -> str:
    """GroundingDINO's text: lower case, each thing ending with a full stop."""
    parts = [" ".join(n.lower().replace(".", " ").split()) for n in names]
    return " ".join(f"{p}." for p in parts if p)


def best_name(label: str, names: Sequence[str]) -> str:
    """The asked-for name a detector label belongs to (the label may be a part of it, or more)."""
    label = label.lower().strip()
    for n in names:
        if n.lower().strip() == label:
            return n
    words = set(label.split())
    scored = sorted(names, key=lambda n: -len(words & set(n.lower().split())))
    return scored[0] if scored and words & set(scored[0].lower().split()) else label


def plausible(box, width: int, height: int, max_share: float = 0.5) -> bool:
    """A box that covers most of the picture, or runs from its left edge to its right edge, is the
    floor or a wall more often than the thing asked for: the detector boxes the whole floor as a
    "desk" when nothing better is there."""
    x0, y0, x1, y1 = box
    if (x1 - x0) * (y1 - y0) > max_share * width * height:
        return False
    return not (x0 <= 2 and x1 >= width - 3)


class Finder:
    def __init__(self, model: str = MODEL, device: str = "auto"):
        self.model_name = model
        self.device = device
        self._model = None
        self._processor = None
        self._lock = threading.Lock()

    @property
    def loaded(self) -> bool:
        return self._model is not None

    def load(self) -> None:
        with self._lock:
            if self._model is not None:
                return
            import torch
            from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

            device = self.device
            if device == "auto":
                device = "cuda" if torch.cuda.is_available() else "cpu"
            revision = REVISION if self.model_name == MODEL else None
            self._processor = AutoProcessor.from_pretrained(self.model_name, revision=revision)
            self._model = AutoModelForZeroShotObjectDetection.from_pretrained(self.model_name, revision=revision).to(device).eval()
            self._torch = torch
            self.device = device

    def find(self, rgb: np.ndarray, names: Sequence[str], min_score: float = 0.0) -> List[Found]:
        """Things in the picture (H x W x 3, RGB) that match `names`, best first."""
        self.load()
        text = prompt(names)
        if not text:
            return []
        threshold = max(min_score, BOX_THRESHOLD) if min_score else BOX_THRESHOLD
        with self._lock, self._torch.no_grad():
            inputs = self._processor(images=rgb, text=text, return_tensors="pt").to(self.device)
            outputs = self._model(**inputs)
            result = self._processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, threshold=threshold, text_threshold=TEXT_THRESHOLD,
                target_sizes=[rgb.shape[:2]])[0]
        labels = result.get("text_labels", result.get("labels"))
        out = []
        h, w = rgb.shape[:2]
        for score, label, box in zip(result["scores"].tolist(), labels, result["boxes"].tolist()):
            if not plausible(box, w, h):
                continue
            out.append(Found(best_name(str(label), names), float(score), tuple(float(v) for v in box)))
        out.sort(key=lambda f: -f.score)
        return out
