try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover
    rospy = None
    Twist = None


class VehicleInterface:
    def __init__(self, config):
        self.config = config
        self.publisher = None
        self.command_topic = config.get("ros.command_topic", "/cmd_vel")
        self.use_twist_cmd = bool(config.get("ros.use_twist_cmd", True))

        if rospy is not None and Twist is not None and self.use_twist_cmd:
            self.publisher = rospy.Publisher(self.command_topic, Twist, queue_size=1)

    def publish(self, command):
        if self.publisher is None:
            return
        self.publisher.publish(self._to_twist(command.steering, command.throttle))

    def publish_stop_or_skip(self):
        self.stop()

    def stop(self):
        if self.publisher is None:
            return
        self.publisher.publish(self._to_twist(0.0, 0.0))

    def _to_twist(self, steering, throttle):
        msg = Twist()
        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)
        return msg
