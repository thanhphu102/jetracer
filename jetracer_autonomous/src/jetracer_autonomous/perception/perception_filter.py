from dataclasses import dataclass, field

from .yolo_detector import LIGHT_CLASSES, SIGN_CLASSES


AVOID_CLASSES = ["avoid_left", "avoid_right"]


@dataclass
class Perception:
    sign: str = None
    sign_confidence: float = 0.0
    sign_bbox: tuple = None
    light: str = None
    light_confidence: float = 0.0
    light_bbox: tuple = None
    avoid: str = None
    avoid_confidence: float = 0.0
    avoid_bbox: tuple = None
    raw_detections: list = field(default_factory=list)


class StableGroup:
    def __init__(self, stable_frames):
        self.stable_frames = int(stable_frames)
        self.candidate_label = None
        self.candidate_count = 0
        self.stable_label = None
        self.stable_confidence = 0.0
        self.stable_bbox = None

    def update(self, detection):
        if detection is None:
            self.candidate_label = None
            self.candidate_count = 0
            self.stable_label = None
            self.stable_confidence = 0.0
            self.stable_bbox = None
            return

        if detection.label == self.candidate_label:
            self.candidate_count += 1
        else:
            self.candidate_label = detection.label
            self.candidate_count = 1

        if self.candidate_count >= self.stable_frames:
            self.stable_label = detection.label
            self.stable_confidence = detection.confidence
            self.stable_bbox = detection.bbox

    def snapshot(self):
        return self.stable_label, self.stable_confidence, self.stable_bbox


class PerceptionFilter:
    def __init__(self, config):
        self.config = config
        self.conf_threshold = float(config.get("model.conf_threshold", 0.6))
        self.sign_group = StableGroup(config.get("perception.sign_stable_frames", 4))
        self.light_group = StableGroup(config.get("perception.light_stable_frames", 3))
        self.avoid_group = StableGroup(config.get("perception.avoid_stable_frames", 2))
        self.last_perception = Perception()

    def update(self, detections):
        detections = detections or []
        sign = self._best_detection(detections, SIGN_CLASSES)
        light = self._best_detection(detections, LIGHT_CLASSES)
        avoid = self._best_detection(detections, AVOID_CLASSES)

        self.sign_group.update(sign)
        self.light_group.update(light)
        self.avoid_group.update(avoid)

        sign_label, sign_conf, sign_bbox = self.sign_group.snapshot()
        light_label, light_conf, light_bbox = self.light_group.snapshot()
        avoid_label, avoid_conf, avoid_bbox = self.avoid_group.snapshot()

        self.last_perception = Perception(
            sign=sign_label,
            sign_confidence=sign_conf,
            sign_bbox=sign_bbox,
            light=light_label,
            light_confidence=light_conf,
            light_bbox=light_bbox,
            avoid=avoid_label,
            avoid_confidence=avoid_conf,
            avoid_bbox=avoid_bbox,
            raw_detections=detections,
        )
        return self.last_perception

    def get_last_stable(self):
        return self.last_perception

    def _best_detection(self, detections, labels):
        candidates = [
            detection
            for detection in detections
            if detection.label in labels and detection.confidence >= self.conf_threshold
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda detection: detection.confidence)
