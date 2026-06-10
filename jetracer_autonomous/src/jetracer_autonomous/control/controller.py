from dataclasses import dataclass

from jetracer_autonomous.decision.decision_manager import (
    AVOID,
    GO_STRAIGHT,
    LOST_LINE,
    RECOVER,
    SIGN_PENDING,
    SLOW_DOWN,
    STOP,
    TURNING_LEFT,
    TURNING_RIGHT,
)

from .steering_utils import clamp, limit_delta, smooth_value
from .turn_controller import NONE, TurnController


@dataclass
class DriveCommand:
    steering: float
    throttle: float
    reason: str
    internal_phase: str = NONE
    maneuver_complete: bool = False


class Controller:
    def __init__(self, config):
        self.config = config
        self.previous_steering = 0.0
        self.turn_controller = TurnController(config)

    def compute_command(self, state_info, line_info, perception, current_time):
        state = state_info.state

        if state in (STOP, LOST_LINE):
            self.turn_controller.reset()
            return self._stop_command(state_info.reason)

        if state == AVOID:
            self.turn_controller.reset()
            return self._avoid_command(state_info)

        if state in (TURNING_LEFT, TURNING_RIGHT, GO_STRAIGHT):
            output = self.turn_controller.update(state_info.turn_action, line_info, current_time)
            self.previous_steering = output.steering
            return DriveCommand(
                steering=output.steering,
                throttle=output.throttle,
                reason=output.reason,
                internal_phase=output.internal_phase,
                maneuver_complete=output.complete,
            )

        self.turn_controller.reset()

        if state in (SIGN_PENDING, SLOW_DOWN, RECOVER):
            throttle = float(self.config.get("throttle.slow", 0.1))
        else:
            throttle = float(self.config.get("throttle.normal", 0.16))

        if not getattr(line_info, "found", False):
            if state == RECOVER:
                return DriveCommand(
                    steering=0.0,
                    throttle=throttle,
                    reason=state_info.reason or "recover_searching_for_line",
                )
            return self._stop_command("line_not_found")

        steering = self._line_follow_steering(line_info)
        return DriveCommand(
            steering=steering,
            throttle=throttle,
            reason=state_info.reason or "follow_line",
        )

    def _line_follow_steering(self, line_info):
        kp = float(self.config.get("line.kp", 0.004))
        max_steering = float(self.config.get("line.max_steering", 0.6))
        alpha = float(self.config.get("control.steering_smoothing_alpha", 0.65))
        max_delta = float(self.config.get("control.max_steering_delta_per_step", 0.1))

        raw = kp * float(line_info.line_error)
        clamped = clamp(raw, -max_steering, max_steering)
        smoothed = smooth_value(clamped, self.previous_steering, alpha)
        limited = limit_delta(smoothed, self.previous_steering, max_delta)
        self.previous_steering = limited
        return limited

    def _avoid_command(self, state_info):
        if state_info.avoid_label == "avoid_left":
            steering = float(self.config.get("steering.avoid_left", 0.45))
            reason = "avoid_left_temporary_override"
        else:
            steering = float(self.config.get("steering.avoid_right", -0.45))
            reason = "avoid_right_temporary_override"

        self.previous_steering = steering
        return DriveCommand(
            steering=steering,
            throttle=float(self.config.get("throttle.slow", 0.1)),
            reason=reason,
        )

    def _stop_command(self, reason):
        self.previous_steering = 0.0
        return DriveCommand(
            steering=0.0,
            throttle=float(self.config.get("throttle.stop", 0.0)),
            reason=reason or "stop",
        )
