#!/usr/bin/env python
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def main():
    rospy.init_node("csi_camera_node")

    topic = rospy.get_param("~image_topic", "/camera/image_raw")
    width = int(rospy.get_param("~width", 1280))
    height = int(rospy.get_param("~height", 720))
    fps = int(rospy.get_param("~fps", 30))
    sensor_id = int(rospy.get_param("~sensor_id", 0))

    pipeline = (
        "nvarguscamerasrc sensor-id={sensor_id} ! "
        "video/x-raw(memory:NVMM),width={width},height={height},"
        "framerate={fps}/1,format=NV12 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false"
    ).format(sensor_id=sensor_id, width=width, height=height, fps=fps)

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        rospy.logfatal("Failed to open CSI camera pipeline")
        return

    bridge = CvBridge()
    publisher = rospy.Publisher(topic, Image, queue_size=1)
    rate = rospy.Rate(fps)

    rospy.loginfo("CSI camera publishing to %s", topic)

    while not rospy.is_shutdown():
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(2.0, "Failed to read CSI camera frame")
            rate.sleep()
            continue

        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera"
        publisher.publish(msg)
        rate.sleep()

    cap.release()


if __name__ == "__main__":
    main()
