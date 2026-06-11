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
        self.car = None
        self.command_topic = config.get("ros.command_topic", "/cmd_vel")
        self.use_twist_cmd = bool(config.get("ros.use_twist_cmd", True))
        self.backend = str(config.get("vehicle.backend", "twist"))

        if self._twist_enabled() and rospy is not None and Twist is not None:
            self.publisher = rospy.Publisher(self.command_topic, Twist, queue_size=1)

        if self._direct_enabled():
            self.car = self._load_direct_car()

    def publish(self, command):
        self._publish_twist(command.steering, command.throttle)
        self._publish_direct(command.steering, command.throttle)

    def publish_stop_or_skip(self):
        self.stop()

    def stop(self):
        self._publish_twist(0.0, 0.0)
        self._publish_direct(0.0, 0.0)

    def status(self):
        return "vehicle.backend={} twist={} direct_jetracer={}".format(
            self.backend,
            self.publisher is not None,
            self.car is not None,
        )

    def _to_twist(self, steering, throttle):
        msg = Twist()
        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)
        return msg

    def _twist_enabled(self):
        return self.use_twist_cmd and self.backend in ("twist", "both")

    def _direct_enabled(self):
        return self.backend in ("direct_jetracer", "both")

    def _publish_twist(self, steering, throttle):
        if self.publisher is None:
            return
        self.publisher.publish(self._to_twist(steering, throttle))

    def _publish_direct(self, steering, throttle):
        if self.car is None:
            return

        steering_value = self._scaled(
            steering,
            "vehicle.steering_gain",
            "vehicle.steering_offset",
        )
        throttle_value = self._scaled(
            throttle,
            "vehicle.throttle_gain",
            "vehicle.throttle_offset",
        )
        self.car.steering = steering_value
        self.car.throttle = throttle_value

    def _scaled(self, value, gain_key, offset_key):
        gain = float(self.config.get(gain_key, 1.0))
        offset = float(self.config.get(offset_key, 0.0))
        scaled = float(value) * gain + offset
        return max(-1.0, min(1.0, scaled))

    def _load_direct_car(self):
        try:
            from jetracer.nvidia_racecar import NvidiaRacecar
        except ImportError as exc:
            self._log("direct JetRacer backend unavailable: {}".format(exc))
            return None

        try:
            car = NvidiaRacecar()
            self._log("direct JetRacer backend ready")
            return car
        except Exception as exc:
            self._log("direct JetRacer backend failed: {}".format(exc))
            return None

    def _log(self, message):
        if rospy is not None:
            rospy.logwarn(message)
        else:
            print(message)
