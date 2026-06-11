#!/usr/bin/env python3
import argparse
import json
import os
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer


def load_nvidia_racecar():
    # The ROS workspace repository is also named "jetracer", which can shadow
    # the Python hardware package. Remove repo paths before importing hardware.
    script_path = os.path.abspath(__file__)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(script_path), "..", ".."))
    catkin_src = os.path.dirname(repo_root)
    sys.path[:] = [
        path
        for path in sys.path
        if os.path.abspath(path or os.getcwd()) not in (repo_root, catkin_src)
    ]

    from jetracer.nvidia_racecar import NvidiaRacecar

    return NvidiaRacecar


class MotorService:
    def __init__(self, steering_gain=1.0, steering_offset=0.0, throttle_gain=1.0, throttle_offset=0.0):
        self.car = load_nvidia_racecar()()
        self.steering_gain = float(steering_gain)
        self.steering_offset = float(steering_offset)
        self.throttle_gain = float(throttle_gain)
        self.throttle_offset = float(throttle_offset)
        self.last = {"steering": 0.0, "throttle": 0.0}

    def drive(self, steering, throttle):
        steering_value = self._scale(steering, self.steering_gain, self.steering_offset)
        throttle_value = self._scale(throttle, self.throttle_gain, self.throttle_offset)
        self.car.steering = steering_value
        self.car.throttle = throttle_value
        self.last = {"steering": steering_value, "throttle": throttle_value}
        return self.last

    def stop(self):
        return self.drive(0.0, 0.0)

    def _scale(self, value, gain, offset):
        scaled = float(value) * gain + offset
        return max(-1.0, min(1.0, scaled))


service = None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/health":
            self._send_json({"ok": True, "last": service.last})
            return
        if self.path == "/stop":
            self._send_json({"ok": True, "command": service.stop()})
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/drive":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            payload = self.rfile.read(length).decode("utf-8")
            data = json.loads(payload or "{}")
            command = service.drive(
                float(data.get("steering", 0.0)),
                float(data.get("throttle", 0.0)),
            )
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, status=400)
            return

        self._send_json({"ok": True, "command": command})

    def log_message(self, fmt, *args):
        return

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    global service

    parser = argparse.ArgumentParser(description="Python 3 JetRacer motor HTTP service.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--steering-gain", type=float, default=1.0)
    parser.add_argument("--steering-offset", type=float, default=0.0)
    parser.add_argument("--throttle-gain", type=float, default=1.0)
    parser.add_argument("--throttle-offset", type=float, default=0.0)
    args = parser.parse_args()

    service = MotorService(
        steering_gain=args.steering_gain,
        steering_offset=args.steering_offset,
        throttle_gain=args.throttle_gain,
        throttle_offset=args.throttle_offset,
    )

    server = HTTPServer((args.host, args.port), Handler)
    print("JetRacer motor HTTP service listening on {}:{}".format(args.host, args.port), flush=True)
    try:
        server.serve_forever()
    finally:
        service.stop()


if __name__ == "__main__":
    main()
