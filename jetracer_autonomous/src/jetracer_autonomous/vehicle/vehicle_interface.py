try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover
    rospy = None
    Twist = None

try:
    import json
    import urllib2
except ImportError:  # pragma: no cover
    try:
        import json
        import urllib.request as urllib_request
    except ImportError:
        json = None
        urllib_request = None
    else:
        urllib2 = None
else:
    urllib_request = None


class VehicleInterface:
    def __init__(self, config):
        self.config = config
        self.publisher = None
        self.car = None
        self.command_topic = config.get("ros.command_topic", "/cmd_vel")
        self.use_twist_cmd = bool(config.get("ros.use_twist_cmd", True))
        self.backend = str(config.get("vehicle.backend", "twist"))
        self.http_url = config.get("vehicle.http_url", "http://127.0.0.1:8766/drive")
        self.http_timeout_sec = float(config.get("vehicle.http_timeout_sec", 0.2))

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
        return "vehicle.backend={} twist={} direct_jetracer={} http_jetracer={}".format(
            self.backend,
            self.publisher is not None,
            self.car is not None,
            self._http_enabled(),
        )

    def _to_twist(self, steering, throttle):
        msg = Twist()
        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)
        return msg

    def _twist_enabled(self):
        return self.use_twist_cmd and self.backend in ("twist", "twist_http", "both")

    def _direct_enabled(self):
        return self.backend in ("direct_jetracer", "both")

    def _http_enabled(self):
        return self.backend in ("http_jetracer", "twist_http", "both")

    def _publish_twist(self, steering, throttle):
        if self.publisher is None:
            return
        self.publisher.publish(self._to_twist(steering, throttle))

    def _publish_direct(self, steering, throttle):
        if self.car is None:
            self._publish_http(steering, throttle)
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

    def _publish_http(self, steering, throttle):
        if not self._http_enabled() or json is None:
            return

        payload = json.dumps(
            {
                "steering": self._scaled(
                    steering,
                    "vehicle.steering_gain",
                    "vehicle.steering_offset",
                ),
                "throttle": self._scaled(
                    throttle,
                    "vehicle.throttle_gain",
                    "vehicle.throttle_offset",
                ),
            }
        )
        try:
            if urllib2 is not None:
                request = urllib2.Request(
                    self.http_url,
                    data=payload,
                    headers={"Content-Type": "application/json"},
                )
                urllib2.urlopen(request, timeout=self.http_timeout_sec).read()
            else:
                request = urllib_request.Request(
                    self.http_url,
                    data=payload.encode("utf-8"),
                    headers={"Content-Type": "application/json"},
                )
                urllib_request.urlopen(request, timeout=self.http_timeout_sec).read()
        except Exception as exc:
            self._log("JetRacer motor HTTP request failed: {}".format(exc))

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
