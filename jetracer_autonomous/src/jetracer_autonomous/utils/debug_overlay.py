import os

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


class DebugOverlay:
    def __init__(self, config):
        self.enabled = bool(config.get("debug.publish_overlay", True))
        self.save_frames = bool(config.get("debug.save_overlay_frames", False))
        self.save_interval = int(config.get("debug.overlay_save_interval", 30))
        self.save_path = config.get("debug.overlay_save_path", "/tmp/jetracer_debug/")
        if self.save_frames:
            os.makedirs(self.save_path, exist_ok=True)

    def draw(self, frame, state_info, line_info, perception, command, yolo_ran):
        if not self.enabled or cv2 is None or frame is None:
            return frame

        overlay = frame.copy()
        self._draw_line_debug(overlay, line_info)
        self._draw_detections(overlay, getattr(perception, "raw_detections", []))
        self._draw_text(overlay, state_info, line_info, perception, command, yolo_ran)
        return overlay

    def maybe_save(self, overlay, frame_counter):
        if not self.save_frames or cv2 is None or overlay is None:
            return
        if frame_counter % max(1, self.save_interval) != 0:
            return
        filename = os.path.join(self.save_path, "frame_{:06d}.jpg".format(frame_counter))
        cv2.imwrite(filename, overlay)

    def _draw_line_debug(self, overlay, line_info):
        debug = getattr(line_info, "debug", {}) or {}
        roi_box = debug.get("roi_box")
        if roi_box:
            x1, y1, x2, y2 = roi_box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (255, 255, 0), 1)

        for box in debug.get("split_boxes", []):
            x1, y1, x2, y2 = box
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (80, 80, 255), 2)

        height = overlay.shape[0]
        image_center = int(getattr(line_info, "image_center", 0) or 0)
        cv2.line(overlay, (image_center, 0), (image_center, height), (255, 0, 0), 1)

        line_center = getattr(line_info, "line_center", None)
        if line_center is not None:
            x = int(line_center)
            cv2.line(overlay, (x, 0), (x, height), (0, 255, 0), 2)

    def _draw_detections(self, overlay, detections):
        for detection in detections:
            x1, y1, x2, y2 = [int(v) for v in detection.bbox]
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 180, 255), 2)
            label = "{} {:.2f}".format(detection.label, detection.confidence)
            cv2.putText(
                overlay,
                label,
                (x1, max(12, y1 - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 180, 255),
                1,
                cv2.LINE_AA,
            )

    def _draw_text(self, overlay, state_info, line_info, perception, command, yolo_ran):
        rows = [
            "state={} saved={} phase={}".format(
                state_info.state, state_info.saved_state, command.internal_phase
            ),
            "pending={} yolo={} reason={}".format(
                state_info.pending_action, yolo_ran, command.reason or state_info.reason
            ),
            "err={} cross={} L/C/R/H={:.2f}/{:.2f}/{:.2f}/{:.2f}".format(
                self._fmt(getattr(line_info, "line_error", None)),
                getattr(line_info, "cross", False),
                getattr(line_info, "left_score", 0.0),
                getattr(line_info, "center_score", 0.0),
                getattr(line_info, "right_score", 0.0),
                getattr(line_info, "horizontal_score", 0.0),
            ),
            "raw_cross={} count={}".format(
                getattr(line_info, "raw_cross", False),
                getattr(line_info, "cross_stable_count", 0),
            ),
            "found={} mask={:.3f}".format(
                getattr(line_info, "found", False),
                getattr(line_info, "mask_score", 0.0),
            ),
            "sign={} light={} avoid={}".format(
                getattr(perception, "sign", None),
                getattr(perception, "light", None),
                getattr(perception, "avoid", None),
            ),
            "steering={:.2f} throttle={:.2f}".format(command.steering, command.throttle),
        ]
        y = 20
        for row in rows:
            cv2.putText(
                overlay,
                row,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (20, 20, 20),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                row,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            y += 20

    def _fmt(self, value):
        if value is None:
            return "None"
        return "{:.1f}".format(value)
