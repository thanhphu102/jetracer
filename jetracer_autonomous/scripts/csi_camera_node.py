#!/usr/bin/env python
import time

import cv2
import rospy
from sensor_msgs.msg import Image


def build_pipeline(sensor_id, capture_width, capture_height, capture_fps, width, height):
    return (
        "nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),width={capture_width},height={capture_height},"
        "framerate={capture_fps}/1,format=NV12 ! "
        "nvvidconv ! video/x-raw,width={width},height={height},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink max-buffers=1 drop=true sync=false"
    ).format(
        sensor_id=sensor_id,
        capture_width=capture_width,
        capture_height=capture_height,
        capture_fps=capture_fps,
        width=width,
        height=height,
    )


def open_capture(pipeline):
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        return None
    return cap


def frame_to_msg(frame):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "camera"
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = "bgr8"
    msg.is_bigendian = False
    msg.step = int(frame.shape[1] * 3)
    msg.data = frame.tobytes()
    return msg


def main():
    rospy.init_node("csi_camera_node")

    topic = rospy.get_param("~image_topic", "/camera/image_raw")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    fps = int(rospy.get_param("~fps", 15))
    sensor_id = int(rospy.get_param("~sensor_id", 0))
    capture_width = int(rospy.get_param("~capture_width", 1280))
    capture_height = int(rospy.get_param("~capture_height", 720))
    capture_fps = int(rospy.get_param("~capture_fps", 60))
    reopen_after_failures = int(rospy.get_param("~reopen_after_failures", 5))

    pipeline = build_pipeline(
        sensor_id=sensor_id,
        capture_width=capture_width,
        capture_height=capture_height,
        capture_fps=capture_fps,
        width=width,
        height=height,
    )

    rospy.loginfo("CSI camera pipeline: %s", pipeline)
    cap = open_capture(pipeline)
    if cap is None:
        rospy.logfatal("Failed to open CSI camera pipeline")
        return

    publisher = rospy.Publisher(topic, Image, queue_size=1)
    rate = rospy.Rate(fps)
    consecutive_failures = 0
    frame_count = 0
    fps_window_start = time.time()

    rospy.loginfo("CSI camera publishing %dx%d to %s", width, height, topic)

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            consecutive_failures += 1
            rospy.logwarn_throttle(
                2.0,
                "Failed to read CSI camera frame; consecutive_failures=%d",
                consecutive_failures,
            )
            if consecutive_failures >= reopen_after_failures:
                rospy.logwarn("Reopening CSI camera pipeline")
                cap.release()
                rospy.sleep(1.0)
                cap = open_capture(pipeline)
                consecutive_failures = 0
                if cap is None:
                    rospy.logerr("Failed to reopen CSI camera pipeline")
                    rospy.sleep(1.0)
            rate.sleep()
            continue

        consecutive_failures = 0
        publisher.publish(frame_to_msg(frame))
        frame_count += 1
        elapsed = time.time() - fps_window_start
        if elapsed >= 5.0:
            rospy.loginfo("CSI camera publish fps=%.2f", frame_count / elapsed)
            frame_count = 0
            fps_window_start = time.time()
        rate.sleep()

    if cap is not None:
        cap.release()


if __name__ == "__main__":
    main()
