from dataclasses import dataclass
import json
import urllib.error
import urllib.request

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


LANE_CLASSES = ["straight", "curve_left", "curve_right", "avoid_left", "avoid_right"]
SIGN_CLASSES = ["0_Go_straight", "1_Turn_left", "2_Turn_right", "3_Prohibited"]
LIGHT_CLASSES = ["green", "yellow", "red_stop", "red_slow"]

LABEL_ALIASES = {
    "go_straight": "0_Go_straight",
    "go straight": "0_Go_straight",
    "straight_sign": "0_Go_straight",
    "0_go_straight": "0_Go_straight",
    "0_go straight": "0_Go_straight",
    "turn_left": "1_Turn_left",
    "turn left": "1_Turn_left",
    "left": "1_Turn_left",
    "left_sign": "1_Turn_left",
    "1_turn_left": "1_Turn_left",
    "1_turn left": "1_Turn_left",
    "turn_right": "2_Turn_right",
    "turn right": "2_Turn_right",
    "right": "2_Turn_right",
    "right_sign": "2_Turn_right",
    "2_turn_right": "2_Turn_right",
    "2_turn right": "2_Turn_right",
    "prohibited": "3_Prohibited",
    "prohibit": "3_Prohibited",
    "no_entry": "3_Prohibited",
    "no entry": "3_Prohibited",
    "stop_sign": "3_Prohibited",
    "3_prohibited": "3_Prohibited",
    "red": "red_stop",
    "red_light": "red_stop",
    "red stop": "red_stop",
    "red_stop": "red_stop",
    "red_slow": "red_slow",
    "red slow": "red_slow",
    "yellow": "yellow",
    "yellow_light": "yellow",
    "green": "green",
    "green_light": "green",
    "avoid_left": "avoid_left",
    "avoid left": "avoid_left",
    "avoid_right": "avoid_right",
    "avoid right": "avoid_right",
}


@dataclass
class Detection:
    label: str
    confidence: float
    bbox: tuple


class YOLODetector:
    """Thin YOLO wrapper. It reports detections and never makes driving decisions."""

    def __init__(
        self,
        model_path,
        conf_threshold=0.6,
        logger=None,
        backend="local",
        http_url="http://127.0.0.1:8765/detect",
        http_timeout_sec=1.0,
        jpeg_quality=80,
    ):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.logger = logger
        self.backend = backend
        self.http_url = http_url
        self.http_timeout_sec = http_timeout_sec
        self.jpeg_quality = int(jpeg_quality)
        self.model = None
        self.names = {}
        if self.backend == "http":
            self._log("Using YOLO HTTP backend: {}".format(self.http_url))
        else:
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
        if self.backend == "http":
            return self._detect_http(frame)

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
                detections.append(
                    Detection(label=normalize_label(label), confidence=confidence, bbox=xyxy)
                )

        return detections

    def _detect_http(self, frame):
        if cv2 is None or frame is None:
            return []

        ok, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self._log("Failed to encode frame for YOLO HTTP backend")
            return []

        request = urllib.request.Request(
            self.http_url,
            data=encoded.tobytes(),
            headers={
                "Content-Type": "image/jpeg",
                "X-Confidence": str(self.conf_threshold),
            },
            method="POST",
        )

        try:
            with urllib.request.urlopen(request, timeout=self.http_timeout_sec) as response:
                payload = response.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            self._log("YOLO HTTP request failed: {}".format(exc))
            return []

        try:
            data = json.loads(payload)
        except ValueError as exc:
            self._log("YOLO HTTP response was not JSON: {}".format(exc))
            return []

        detections = []
        for item in data.get("detections", []):
            try:
                detections.append(
                    Detection(
                        label=normalize_label(str(item["label"])),
                        confidence=float(item["confidence"]),
                        bbox=tuple(float(v) for v in item["bbox"]),
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue
        return detections

    def _log(self, message):
        if self.logger:
            self.logger(message)
        else:
            print(message)


def normalize_label(label):
    if label in SIGN_CLASSES or label in LIGHT_CLASSES or label in LANE_CLASSES:
        return label

    key = label.strip().lower().replace("-", "_")
    return LABEL_ALIASES.get(key, label)
