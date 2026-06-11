#!/usr/bin/env python3
import argparse
import json
import threading
import time
from urllib.parse import urlparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from ultralytics import YOLO

try:
    import torch
except ImportError:  # pragma: no cover
    torch = None


class YOLOService:
    def __init__(self, model_path, default_conf, device=None, imgsz=640, half=False):
        self.model = YOLO(model_path)
        self.default_conf = default_conf
        self.device = device
        self.imgsz = imgsz
        self.half = half
        self.cuda_available = bool(torch is not None and torch.cuda.is_available())
        if self.cuda_available:
            self.cuda_device_name = torch.cuda.get_device_name(0)
        else:
            self.cuda_device_name = None
        self.lock = threading.Lock()
        self.model_names = getattr(self.model, "names", {}) or {}
        self.last_image_bytes = None
        self.last_detections = []
        self.last_error = None
        self.last_conf = None
        self.last_inference_ms = None
        self.request_count = 0

    def detect(self, image_bytes, conf=None):
        self.last_image_bytes = image_bytes
        self.last_error = None
        self.request_count += 1
        array = np.frombuffer(image_bytes, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            self.last_error = "cv2.imdecode returned None"
            self.last_detections = []
            return []

        threshold = self.default_conf if conf is None else conf
        self.last_conf = threshold
        predict_kwargs = {
            "conf": threshold,
            "verbose": False,
            "imgsz": self.imgsz,
            "half": self.half,
        }
        if self.device:
            predict_kwargs["device"] = self.device
        started_at = time.time()
        with self.lock:
            results = self.model(frame, **predict_kwargs)
        self.last_inference_ms = (time.time() - started_at) * 1000.0
        detections = []

        for result in results:
            names = getattr(result, "names", {}) or {}
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

                detections.append(
                    {
                        "label": label,
                        "confidence": float(box.conf[0].item()),
                        "bbox": [float(v) for v in box.xyxy[0].tolist()],
                    }
                )

        self.last_detections = detections
        return detections


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/health":
                self._send_json(service.health_payload())
                return
            if path == "/last.json":
                self._send_json(service.last_payload())
                return
            if path == "/last.jpg":
                if service.last_image_bytes is None:
                    self.send_error(404, "No image has been received yet")
                    return
                self._send_bytes(service.last_image_bytes, "image/jpeg")
                return
            if path == "/names":
                self._send_json({"names": service.model_names})
                return
            if path != "/health":
                self.send_error(404)
                return

        def do_POST(self):
            if self.path != "/detect":
                self.send_error(404)
                return

            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = 0

            image_bytes = self.rfile.read(length)
            conf = self._read_confidence()

            try:
                detections = service.detect(image_bytes, conf=conf)
            except Exception as exc:
                service.last_error = str(exc)
                self.send_error(500, "YOLO inference failed: {}".format(exc))
                return

            self._send_json({"detections": detections})

        def log_message(self, fmt, *args):
            print("{} - {}".format(self.address_string(), fmt % args))

        def _read_confidence(self):
            value = self.headers.get("X-Confidence")
            if value is None:
                return None
            try:
                return float(value)
            except ValueError:
                return None

        def _send_json(self, payload):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body, content_type):
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return Handler


def _jsonable_names(names):
    if isinstance(names, dict):
        return {str(key): value for key, value in names.items()}
    if isinstance(names, (list, tuple)):
        return {str(index): value for index, value in enumerate(names)}
    return {}


def _service_health_payload(self):
    return {
        "ok": True,
        "device": self.device,
        "imgsz": self.imgsz,
        "half": self.half,
        "conf": self.default_conf,
        "cuda_available": self.cuda_available,
        "cuda_device_name": self.cuda_device_name,
        "request_count": self.request_count,
        "last_conf": self.last_conf,
        "last_inference_ms": self.last_inference_ms,
        "last_error": self.last_error,
        "last_detection_count": len(self.last_detections),
        "names": _jsonable_names(self.model_names),
    }


def _service_last_payload(self):
    return {
        "request_count": self.request_count,
        "last_conf": self.last_conf,
        "last_inference_ms": self.last_inference_ms,
        "last_error": self.last_error,
        "detections": self.last_detections,
    }


YOLOService.health_payload = _service_health_payload
YOLOService.last_payload = _service_last_payload


def main():
    parser = argparse.ArgumentParser(description="Small YOLO HTTP detection service.")
    parser.add_argument("--model", required=True, help="Path to YOLO .pt model")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host")
    parser.add_argument("--port", type=int, default=8765, help="Bind port")
    parser.add_argument("--conf", type=float, default=0.6, help="Default confidence threshold")
    parser.add_argument("--device", default="0", help="Ultralytics device, e.g. 0 or cpu")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference image size")
    parser.add_argument("--half", action="store_true", help="Use FP16 inference where supported")
    args = parser.parse_args()

    service = YOLOService(args.model, args.conf, device=args.device, imgsz=args.imgsz, half=args.half)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(service))
    print(
        "YOLO HTTP service listening on {}:{} model={} device={} imgsz={} half={}".format(
            args.host, args.port, args.model, args.device, args.imgsz, args.half
        )
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
