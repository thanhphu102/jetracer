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
        self.enabled_signs = self._enabled_signs()
        self.sign_group = StableGroup(config.get("perception.sign_stable_frames", 4))
        self.light_group = StableGroup(config.get("perception.light_stable_frames", 3))
        self.avoid_group = StableGroup(config.get("perception.avoid_stable_frames", 2))
        self.last_perception = Perception()

    def update(self, detections, frame_shape=None):
        detections = detections or []
        sign = self._best_detection(
            detections,
            self.enabled_signs,
            kind="sign",
            frame_shape=frame_shape,
        )
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

    def _enabled_signs(self):
        configured = self.config.get("sign.enabled_signs", None)
        if isinstance(configured, list) and configured:
            enabled = [label for label in configured if label in SIGN_CLASSES]
        else:
            enabled = list(SIGN_CLASSES)

        if not bool(self.config.get("sign.prohibited_enabled", True)):
            enabled = [label for label in enabled if label != "3_Prohibited"]
        return enabled

    def _best_detection(self, detections, labels, kind=None, frame_shape=None):
        candidates = [
            detection
            for detection in detections
            if detection.label in labels
            and detection.confidence >= self._threshold_for(detection.label, kind)
            and self._inside_roi(detection, kind, frame_shape)
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda detection: detection.confidence)

    def _threshold_for(self, label, kind=None):
        if kind == "sign":
            class_threshold = self.config.get("sign.class_thresholds.{}".format(label), None)
            if class_threshold is not None:
                return float(class_threshold)
        return self.conf_threshold

    def _inside_roi(self, detection, kind=None, frame_shape=None):
        if kind != "sign" or frame_shape is None:
            return True

        try:
            height = float(frame_shape[0])
            _, y1, _, y2 = detection.bbox
            center_y_fraction = ((float(y1) + float(y2)) * 0.5) / height
        except (TypeError, ValueError, ZeroDivisionError, IndexError):
            return False

        min_fraction = float(self.config.get("sign.roi_y_min_fraction", 0.0))
        max_fraction = float(self.config.get("sign.roi_y_max_fraction", 1.0))
        return min_fraction <= center_y_fraction <= max_fraction
