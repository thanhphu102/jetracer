from dataclasses import dataclass


LANE_CLASSES = ["straight", "curve_left", "curve_right", "avoid_left", "avoid_right"]
SIGN_CLASSES = ["0_Go_straight", "1_Turn_left", "2_Turn_right", "3_Prohibited"]
LIGHT_CLASSES = ["green", "yellow", "red_stop", "red_slow"]


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: tuple


class YOLODetector:
    """Thin YOLO wrapper. It reports detections and never makes driving decisions."""

    def __init__(self, model_path, conf_threshold=0.6, logger=None):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.logger = logger
        self.model = None
        self.names = {}
        self._load_model()

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self._log("ultralytics is not installed; YOLO detection is disabled")
            return

        try:
            self.model = YOLO(self.model_path)
            self.names = getattr(self.model, "names", {}) or {}
            self._log("Loaded YOLO model: {}".format(self.model_path))
        except Exception as exc:  # pragma: no cover - hardware/model dependent
            self.model = None
            self._log("Failed to load YOLO model {}: {}".format(self.model_path, exc))

    def detect(self, frame):
        if self.model is None or frame is None:
            return []

        try:
            results = self.model(frame, conf=self.conf_threshold, verbose=False)
        except Exception as exc:  # pragma: no cover
            self._log("YOLO inference failed: {}".format(exc))
            return []

        detections = []
        for result in results:
            names = getattr(result, "names", None) or self.names
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue

            for box in boxes:
                cls_idx = int(box.cls[0].item())
                if isinstance(names, dict):
                    label = names.get(cls_idx, str(cls_idx))
                elif isinstance(names, (list, tuple)) and cls_idx < len(names):
                    label = names[cls_idx]
                else:
                    label = str(cls_idx)
                confidence = float(box.conf[0].item())
                xyxy = tuple(float(v) for v in box.xyxy[0].tolist())
                detections.append(Detection(label=label, confidence=confidence, bbox=xyxy))

        return detections

    def _log(self, message):
        if self.logger:
            self.logger(message)
        else:
            print(message)
