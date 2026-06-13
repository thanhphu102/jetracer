#!/usr/bin/env python3
"""
ROS turn controller for JetRacer steering/throttle control.

This node ports the notebook turn state machine into rospy.

Inputs:
- `~lane_cmd_topic` (`geometry_msgs/Twist`)
  `linear.x` = throttle, `angular.z` = steering from the lane follower.
- `~lane_status_topic` (`std_msgs/String`)
  JSON: `{"ok": true, "angle_deg": -3.0, "lateral_error": 12.0}`
- `~intersection_status_topic` (`std_msgs/String`)
  JSON: `{"is_intersection": true, "raw_intersection": true,
          "branch_left": false, "branch_right": true, "straight": false}`
- `~turn_direction_topic` (`std_msgs/String`)
  One of: `left`, `right`, `straight`, `none`.
- `~imu_topic` (`sensor_msgs/Imu`)

Outputs:
- `~cmd_out_topic` (`geometry_msgs/Twist`)
- `~state_topic` (`std_msgs/String`) JSON debug state
"""

import json
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def wrap_angle_deg(angle_deg):
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def quat_to_yaw_deg(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


class ImuYawTracker:
    def __init__(self):
        self.current_abs_yaw_deg = None
        self.base_yaw_deg = None

    def handle_imu(self, msg):
        q = msg.orientation
        self.current_abs_yaw_deg = quat_to_yaw_deg(q.x, q.y, q.z, q.w)

    def available(self):
        return self.current_abs_yaw_deg is not None

    def reset(self):
        if self.current_abs_yaw_deg is not None:
            self.base_yaw_deg = self.current_abs_yaw_deg

    def relative_yaw_deg(self):
        if self.current_abs_yaw_deg is None or self.base_yaw_deg is None:
            return None
        return wrap_angle_deg(self.current_abs_yaw_deg - self.base_yaw_deg)


class TurnControllerNode:
    def __init__(self):
        self.loop_hz = float(rospy.get_param("~loop_hz", 25.0))
        self.cmd_timeout_sec = float(rospy.get_param("~cmd_timeout_sec", 0.40))
        self.status_timeout_sec = float(rospy.get_param("~status_timeout_sec", 0.60))
        self.intersection_timeout_sec = float(rospy.get_param("~intersection_timeout_sec", 0.60))

        self.turn_left_steering = float(rospy.get_param("~turn_left_steering", -0.85))
        self.turn_right_steering = float(rospy.get_param("~turn_right_steering", 0.95))
        self.turn_throttle = float(rospy.get_param("~turn_throttle", 0.12))
        self.approach_throttle = float(rospy.get_param("~approach_throttle", 0.11))
        self.realign_throttle_max = float(rospy.get_param("~realign_throttle_max", 0.10))
        self.lost_throttle = float(rospy.get_param("~lost_throttle", 0.02))

        self.approach_sec = float(rospy.get_param("~approach_sec", 0.25))
        self.turn_left_min_sec = float(rospy.get_param("~turn_left_min_sec", 0.55))
        self.turn_right_min_sec = float(rospy.get_param("~turn_right_min_sec", 0.65))
        self.turn_left_max_sec = float(rospy.get_param("~turn_left_max_sec", 2.10))
        self.turn_right_max_sec = float(rospy.get_param("~turn_right_max_sec", 2.40))
        self.turn_left_target_deg = float(rospy.get_param("~turn_left_target_deg", 82.0))
        self.turn_right_target_deg = float(rospy.get_param("~turn_right_target_deg", 82.0))
        self.turn_gyro_tol_deg = float(rospy.get_param("~turn_gyro_tol_deg", 4.0))
        self.straight_cross_sec = float(rospy.get_param("~straight_cross_sec", 0.70))

        self.realign_timeout_sec = float(rospy.get_param("~realign_timeout_sec", 2.5))
        self.realign_stable_frames = int(rospy.get_param("~realign_stable_frames", 6))
        self.realign_max_abs_lat = float(rospy.get_param("~realign_max_abs_lat", 24.0))
        self.realign_max_abs_angle = float(rospy.get_param("~realign_max_abs_angle", 12.0))
        self.post_turn_rearm_sec = float(rospy.get_param("~post_turn_rearm_sec", 4.0))
        self.rearm_clear_frames = int(rospy.get_param("~rearm_clear_frames", 8))

        self.allowed_turns = {"left", "right", "straight", "none"}
        self.turn_direction = rospy.get_param("~default_turn_direction", "right").strip().lower()
        if self.turn_direction not in self.allowed_turns:
            self.turn_direction = "right"

        self.state = "FOLLOW"
        self.state_start = rospy.get_time()
        self.rearm_required = False
        self.post_turn_ignore_until = 0.0
        self.intersection_clear_frames = self.rearm_clear_frames
        self.relock_frames = 0

        self.yaw_tracker = ImuYawTracker()
        self.last_lane_cmd = Twist()
        self.last_lane_cmd_time = 0.0
        self.last_lane_status = {}
        self.last_lane_status_time = 0.0
        self.last_intersection_status = {}
        self.last_intersection_time = 0.0
        self.last_output_steering = 0.0
        self.last_output_throttle = 0.0

        lane_cmd_topic = rospy.get_param("~lane_cmd_topic", "/lane/cmd")
        lane_status_topic = rospy.get_param("~lane_status_topic", "/lane/status")
        intersection_status_topic = rospy.get_param("~intersection_status_topic", "/intersection/status")
        turn_direction_topic = rospy.get_param("~turn_direction_topic", "/turn_direction")
        imu_topic = rospy.get_param("~imu_topic", "/imu")
        cmd_out_topic = rospy.get_param("~cmd_out_topic", "/cmd_vel")
        state_topic = rospy.get_param("~state_topic", "/turn_controller/state")

        self.cmd_pub = rospy.Publisher(cmd_out_topic, Twist, queue_size=1)
        self.state_pub = rospy.Publisher(state_topic, String, queue_size=1)

        rospy.Subscriber(lane_cmd_topic, Twist, self.on_lane_cmd, queue_size=1)
        rospy.Subscriber(lane_status_topic, String, self.on_lane_status, queue_size=1)
        rospy.Subscriber(intersection_status_topic, String, self.on_intersection_status, queue_size=1)
        rospy.Subscriber(turn_direction_topic, String, self.on_turn_direction, queue_size=1)
        rospy.Subscriber(imu_topic, Imu, self.on_imu, queue_size=20)

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(1.0, self.loop_hz)), self.on_timer)

        rospy.loginfo("turn_controller_node ready turn=%s imu=%s cmd_out=%s", self.turn_direction, imu_topic, cmd_out_topic)

    def parse_json_msg(self, msg):
        try:
            return json.loads(msg.data)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "JSON parse failed: %s", exc)
            return None

    def on_lane_cmd(self, msg):
        self.last_lane_cmd = msg
        self.last_lane_cmd_time = rospy.get_time()

    def on_lane_status(self, msg):
        data = self.parse_json_msg(msg)
        if data is None:
            return
        self.last_lane_status = data
        self.last_lane_status_time = rospy.get_time()

    def on_intersection_status(self, msg):
        data = self.parse_json_msg(msg)
        if data is None:
            return
        self.last_intersection_status = data
        self.last_intersection_time = rospy.get_time()

    def on_turn_direction(self, msg):
        value = msg.data.strip().lower()
        if value not in self.allowed_turns:
            rospy.logwarn("Unsupported turn direction: %s", value)
            return
        self.turn_direction = value
        rospy.loginfo("turn_direction -> %s", self.turn_direction)

    def on_imu(self, msg):
        self.yaw_tracker.handle_imu(msg)

    def lane_cmd_fresh(self):
        return (rospy.get_time() - self.last_lane_cmd_time) <= self.cmd_timeout_sec

    def lane_status_fresh(self):
        return (rospy.get_time() - self.last_lane_status_time) <= self.status_timeout_sec

    def intersection_fresh(self):
        return (rospy.get_time() - self.last_intersection_time) <= self.intersection_timeout_sec

    def enter_state(self, new_state):
        self.state = new_state
        self.state_start = rospy.get_time()
        rospy.loginfo("STATE -> %s", new_state)

    def publish_command(self, steering, throttle, reason=""):
        cmd = Twist()
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

        self.last_output_steering = cmd.angular.z
        self.last_output_throttle = cmd.linear.x
        self.publish_state(reason)

    def publish_state(self, reason=""):
        state_msg = {
            "state": self.state,
            "turn_direction": self.turn_direction,
            "yaw_deg": self.yaw_tracker.relative_yaw_deg(),
            "rearm_required": self.rearm_required,
            "intersection_clear_frames": self.intersection_clear_frames,
            "lane_cmd_fresh": self.lane_cmd_fresh(),
            "lane_status_fresh": self.lane_status_fresh(),
            "intersection_fresh": self.intersection_fresh(),
            "output_steering": self.last_output_steering,
            "output_throttle": self.last_output_throttle,
            "reason": reason,
        }
        self.state_pub.publish(String(data=json.dumps(state_msg)))

    def stop_and_hold(self, reason):
        rospy.logwarn_throttle(1.0, reason)
        self.publish_command(0.0, 0.0, reason)

    def get_lane_status(self):
        return self.last_lane_status if self.lane_status_fresh() else {}

    def get_intersection_status(self):
        return self.last_intersection_status if self.intersection_fresh() else {}

    def update_rearm(self, now, inter):
        if not self.rearm_required:
            return

        if bool(inter.get("raw_intersection", False)):
            self.intersection_clear_frames = 0
        else:
            self.intersection_clear_frames = min(self.rearm_clear_frames, self.intersection_clear_frames + 1)

        if now < self.post_turn_ignore_until:
            return

        if self.intersection_clear_frames >= self.rearm_clear_frames:
            self.rearm_required = False
            rospy.loginfo("rearm cleared")

    def should_trigger_turn(self, inter):
        if self.turn_direction == "none":
            return False
        if self.state != "FOLLOW" or self.rearm_required:
            return False
        if not bool(inter.get("is_intersection", False)):
            return False

        if self.turn_direction == "left":
            return bool(inter.get("branch_left", False))
        if self.turn_direction == "right":
            return bool(inter.get("branch_right", False))
        return bool(inter.get("straight", False))

    def lane_realign_ok(self, lane_status):
        if not bool(lane_status.get("ok", False)):
            return False
        angle_deg = abs(float(lane_status.get("angle_deg", 999.0)))
        lateral_error = abs(float(lane_status.get("lateral_error", 999.0)))
        return angle_deg < self.realign_max_abs_angle and lateral_error < self.realign_max_abs_lat

    def on_timer(self, _event):
        now = rospy.get_time()
        inter = self.get_intersection_status()
        lane_status = self.get_lane_status()
        self.update_rearm(now, inter)

        if self.state == "FOLLOW":
            if self.should_trigger_turn(inter):
                self.publish_command(0.0, 0.0, "intersection_found")
                self.enter_state("APPROACH")
                return

            if self.lane_cmd_fresh():
                self.publish_command(self.last_lane_cmd.angular.z, self.last_lane_cmd.linear.x, "follow_passthrough")
            else:
                self.stop_and_hold("lane_cmd timeout in FOLLOW")
            return

        if self.state == "APPROACH":
            self.publish_command(0.0, self.approach_throttle, "approach")
            if (now - self.state_start) >= self.approach_sec:
                if self.turn_direction == "left":
                    self.yaw_tracker.reset()
                    self.enter_state("TURN_LEFT")
                elif self.turn_direction == "right":
                    self.yaw_tracker.reset()
                    self.enter_state("TURN_RIGHT")
                else:
                    self.enter_state("STRAIGHT")
            return

        if self.state == "TURN_LEFT":
            if not self.yaw_tracker.available():
                self.stop_and_hold("imu unavailable in TURN_LEFT")
                return
            self.publish_command(self.turn_left_steering, self.turn_throttle, "turn_left")
            elapsed = now - self.state_start
            yaw_deg = self.yaw_tracker.relative_yaw_deg()
            if (
                yaw_deg is not None
                and elapsed >= self.turn_left_min_sec
                and abs(yaw_deg) >= (self.turn_left_target_deg - self.turn_gyro_tol_deg)
            ):
                self.rearm_required = True
                self.post_turn_ignore_until = now + self.post_turn_rearm_sec
                self.intersection_clear_frames = 0
                self.relock_frames = 0
                self.enter_state("REALIGN")
                return
            if elapsed >= self.turn_left_max_sec:
                self.stop_and_hold("turn-left timeout before gyro target")
            return

        if self.state == "TURN_RIGHT":
            if not self.yaw_tracker.available():
                self.stop_and_hold("imu unavailable in TURN_RIGHT")
                return
            self.publish_command(self.turn_right_steering, self.turn_throttle, "turn_right")
            elapsed = now - self.state_start
            yaw_deg = self.yaw_tracker.relative_yaw_deg()
            if (
                yaw_deg is not None
                and elapsed >= self.turn_right_min_sec
                and abs(yaw_deg) >= (self.turn_right_target_deg - self.turn_gyro_tol_deg)
            ):
                self.rearm_required = True
                self.post_turn_ignore_until = now + self.post_turn_rearm_sec
                self.intersection_clear_frames = 0
                self.relock_frames = 0
                self.enter_state("REALIGN")
                return
            if elapsed >= self.turn_right_max_sec:
                self.stop_and_hold("turn-right timeout before gyro target")
            return

        if self.state == "STRAIGHT":
            self.publish_command(0.0, self.approach_throttle, "straight_cross")
            if (now - self.state_start) >= self.straight_cross_sec:
                self.rearm_required = True
                self.post_turn_ignore_until = now + self.post_turn_rearm_sec
                self.intersection_clear_frames = 0
                self.relock_frames = 0
                self.enter_state("REALIGN")
            return

        if self.state == "REALIGN":
            if self.lane_cmd_fresh():
                steering = float(self.last_lane_cmd.angular.z)
                throttle = min(float(self.last_lane_cmd.linear.x), self.realign_throttle_max)
            else:
                steering = clamp(self.last_output_steering, -0.75, 0.75)
                throttle = self.lost_throttle

            self.publish_command(steering, throttle, "realign")

            if self.lane_realign_ok(lane_status):
                self.relock_frames += 1
            else:
                self.relock_frames = 0

            if self.relock_frames >= self.realign_stable_frames:
                self.enter_state("FOLLOW")
                return

            if (now - self.state_start) > self.realign_timeout_sec:
                self.stop_and_hold("realign timeout")
            return

        self.stop_and_hold("unknown state")


def main():
    rospy.init_node("turn_controller_node")
    TurnControllerNode()
    rospy.spin()


if __name__ == "__main__":
    main()
