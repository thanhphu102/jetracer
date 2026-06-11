import os
import threading
import time

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

try:
    import rospy
    from sensor_msgs.msg import Image
except ImportError:  # pragma: no cover
    rospy = None
    Image = None

from jetracer_autonomous.config import Config, default_config_path
from jetracer_autonomous.control.controller import Controller
from jetracer_autonomous.decision.decision_manager import DecisionManager
from jetracer_autonomous.perception.line_detector import LineDetector
from jetracer_autonomous.perception.perception_filter import Perception, PerceptionFilter
from jetracer_autonomous.perception.yolo_detector import YOLODetector
from jetracer_autonomous.utils.debug_overlay import DebugOverlay
from jetracer_autonomous.utils.logger import DebugLogger
from jetracer_autonomous.vehicle.vehicle_interface import VehicleInterface


class AutonomousDriveNode:
    def __init__(self, config_path):
        self.config = Config.load(config_path)
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = None
        self.frame_counter = 0
        self.encoding_logged = False
        self.log_raw_yolo = bool(self.config.get("debug.log_raw_yolo", False))

        self.line_detector = LineDetector(self.config)
        self.perception_filter = PerceptionFilter(self.config)
        self.decision_manager = DecisionManager(self.config)
        self.controller = Controller(self.config)
        self.vehicle = VehicleInterface(self.config)
        self.overlay = DebugOverlay(self.config)
        self.debug_logger = DebugLogger(self.config, ros_log=rospy.loginfo)

        model_path = self.config.resolve_path(self.config.get("model.path", "models/best.pt"))
        self.yolo_detector = YOLODetector(
            model_path=model_path,
            conf_threshold=float(self.config.get("model.conf_threshold", 0.6)),
            logger=rospy.loginfo,
            backend=self.config.get("model.backend", "local"),
            http_url=self.config.get("model.http_url", "http://127.0.0.1:8765/detect"),
            http_timeout_sec=float(self.config.get("model.http_timeout_sec", 1.0)),
            jpeg_quality=int(self.config.get("model.jpeg_quality", 80)),
        )

        self.overlay_pub = None
        if bool(self.config.get("debug.publish_overlay", True)):
            self.overlay_pub = rospy.Publisher("~debug_overlay", Image, queue_size=1)
        self.line_mask_pub = None
        if bool(self.config.get("debug.publish_line_mask", True)):
            self.line_mask_pub = rospy.Publisher("~line_mask", Image, queue_size=1)

        camera_topic = self.config.get("ros.camera_topic", "/camera/image_raw")
        rospy.Subscriber(camera_topic, Image, self._camera_callback, queue_size=1, buff_size=2**24)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Autonomous node started")
        rospy.loginfo("camera_topic={}".format(camera_topic))
        rospy.loginfo("command_topic={}".format(self.config.get("ros.command_topic", "/cmd_vel")))
        rospy.loginfo("model_path={}".format(model_path))
        rospy.loginfo(
            "throttle.normal={} steering.turn_left={} steering.turn_right={}".format(
                self.config.get("throttle.normal", 0.16),
                self.config.get("steering.turn_left", 0.7),
                self.config.get("steering.turn_right", -0.7),
            )
        )
        rospy.loginfo(
            "opencv_cuda_requested={} opencv_cuda_available={}".format(
                self.config.get("line.use_cuda", False),
                getattr(self.line_detector, "cuda_available", False),
            )
        )

    def run(self):
        loop_rate = float(self.config.get("ros.loop_rate_hz", 15))
        inference_every = max(1, int(self.config.get("model.inference_every_n_frames", 3)))
        dry_run = bool(self.config.get("debug.dry_run", True))
        debug_every = max(1, int(self.config.get("debug.publish_debug_every_n_frames", 1)))
        rate = rospy.Rate(loop_rate)
        perception = Perception()

        while not rospy.is_shutdown():
            now = time.time()
            frame = self._get_latest_frame()

            if frame is None:
                self.vehicle.stop()
                rate.sleep()
                continue

            line_info = self.line_detector.process(frame)

            yolo_ran = False
            if self.frame_counter % inference_every == 0:
                detections = self.yolo_detector.detect(frame)
                if self.log_raw_yolo:
                    self._log_raw_yolo(detections)
                perception = self.perception_filter.update(detections, frame.shape)
                yolo_ran = True
            else:
                perception = self.perception_filter.get_last_stable()

            state_info = self.decision_manager.update(
                line_info=line_info,
                perception=perception,
                current_time=now,
                camera_ok=True,
            )
            command = self.controller.compute_command(state_info, line_info, perception, now)

            if command.maneuver_complete:
                self.decision_manager.notify_maneuver_complete(now)
                state_info = self.decision_manager.get_state_info()

            if dry_run:
                self.vehicle.publish_stop_or_skip()
            else:
                self.vehicle.publish(command)

            self.debug_logger.log(state_info, line_info, perception, command, yolo_ran)
            if self.frame_counter % debug_every == 0:
                self._publish_overlay(frame, state_info, line_info, perception, command, yolo_ran)
                self._publish_line_mask(line_info)

            self.frame_counter += 1
            rate.sleep()

    def shutdown(self):
        self.vehicle.stop()

    def _log_raw_yolo(self, detections):
        if not detections:
            rospy.loginfo("raw_yolo=[]")
            return

        parts = []
        for detection in detections:
            parts.append(
                "{}:{:.2f}@{}".format(
                    detection.label,
                    detection.confidence,
                    tuple(int(v) for v in detection.bbox),
                )
            )
        rospy.loginfo("raw_yolo=[{}]".format(", ".join(parts)))

    def _camera_callback(self, msg):
        if not self.encoding_logged:
            rospy.loginfo("camera encoding={}".format(getattr(msg, "encoding", "unknown")))
            self.encoding_logged = True

        try:
            frame = self._image_msg_to_bgr(msg)
        except Exception as exc:
            rospy.logerr("image conversion failed: {}".format(exc))
            return

        with self.frame_lock:
            self.latest_frame = frame.copy()
            self.latest_stamp = getattr(msg.header, "stamp", None)

    def _get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def _publish_overlay(self, frame, state_info, line_info, perception, command, yolo_ran):
        overlay = self.overlay.draw(frame, state_info, line_info, perception, command, yolo_ran)
        self.overlay.maybe_save(overlay, self.frame_counter)

        if self.overlay_pub is None or overlay is None:
            return
        try:
            self.overlay_pub.publish(self._bgr_to_image_msg(overlay))
        except Exception as exc:
            rospy.logwarn("debug overlay publish failed: {}".format(exc))

    def _publish_line_mask(self, line_info):
        if self.line_mask_pub is None:
            return
        mask = getattr(line_info, "debug", {}).get("mask")
        if mask is None:
            return
        try:
            self.line_mask_pub.publish(self._mono_to_image_msg(mask, "line_mask"))
        except Exception as exc:
            rospy.logwarn("line mask publish failed: {}".format(exc))

    def _image_msg_to_bgr(self, msg):
        if cv2 is None:
            raise RuntimeError("cv2 is required for image conversion")

        encoding = getattr(msg, "encoding", "bgr8")
        height = int(msg.height)
        width = int(msg.width)
        step = int(msg.step)
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ("bgr8", "rgb8"):
            channels = 3
        elif encoding in ("bgra8", "rgba8"):
            channels = 4
        elif encoding == "mono8":
            channels = 1
        else:
            raise ValueError("unsupported image encoding: {}".format(encoding))

        row_width = width * channels
        rows = data.reshape((height, step))[:, :row_width]

        if channels == 1:
            frame = rows.reshape((height, width))
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        frame = rows.reshape((height, width, channels))
        if encoding == "rgb8":
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if encoding == "rgba8":
            return cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        if encoding == "bgra8":
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame.copy()

    def _bgr_to_image_msg(self, frame):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "debug_overlay"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        return msg

    def _mono_to_image_msg(self, frame, frame_id):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1])
        msg.data = frame.tobytes()
        return msg


def main():
    if rospy is None:
        raise RuntimeError("rospy is not available. Run this node inside a ROS Python environment.")

    rospy.init_node("jetracer_autonomous_drive")
    config_path = rospy.get_param("~config_path", default_config_path())
    config_path = os.path.abspath(config_path)
    node = AutonomousDriveNode(config_path)
    node.run()
