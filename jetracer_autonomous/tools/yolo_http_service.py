#!/usr/bin/env python3
import argparse
import json
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

    def detect(self, image_bytes, conf=None):
        array = np.frombuffer(image_bytes, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            return []

        threshold = self.default_conf if conf is None else conf
        predict_kwargs = {
            "conf": threshold,
            "verbose": False,
            "imgsz": self.imgsz,
            "half": self.half,
        }
        if self.device:
            predict_kwargs["device"] = self.device
        results = self.model(frame, **predict_kwargs)
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

        return detections


def make_handler(service):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path != "/health":
                self.send_error(404)
                return
            self._send_json(
                {
                    "ok": True,
                    "device": service.device,
                    "imgsz": service.imgsz,
                    "half": service.half,
                    "cuda_available": service.cuda_available,
                    "cuda_device_name": service.cuda_device_name,
                }
            )

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

    return Handler


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
