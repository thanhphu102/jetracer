from dataclasses import dataclass


FOLLOW_LINE = "FOLLOW_LINE"
SIGN_PENDING = "SIGN_PENDING"
TURNING_LEFT = "TURNING_LEFT"
TURNING_RIGHT = "TURNING_RIGHT"
GO_STRAIGHT = "GO_STRAIGHT"
RECOVER = "RECOVER"
SLOW_DOWN = "SLOW_DOWN"
STOP = "STOP"
AVOID = "AVOID"
LOST_LINE = "LOST_LINE"

TURN_STATES = {TURNING_LEFT, TURNING_RIGHT, GO_STRAIGHT}
SLOW_LIGHTS = {"yellow", "red_slow"}
SIGN_ACTIONS = {
    "0_Go_straight": "STRAIGHT",
    "1_Turn_left": "LEFT",
    "2_Turn_right": "RIGHT",
}


@dataclass
class StateInfo:
    state: str
    saved_state: str = None
    pending_action: str = None
    reason: str = ""
    internal_phase: str = "NONE"
    avoid_label: str = None
    turn_action: str = None
    cooldown_active: bool = False


class DecisionManager:
    def __init__(self, config):
        self.config = config
        self.state = FOLLOW_LINE
        self.pending_action = None
        self.sign_pending_start_time = None
        self.lost_line_since = None
        self.recover_start_time = None
        self.cooldown_until = 0.0

        self.slow_saved_state = None
        self.avoid_saved_state = None
        self.avoid_start_time = None
        self.avoid_label = None
        self.blocked_avoid_label = None

        self.last_reason = "startup"

    def update(self, line_info, perception, current_time, camera_ok=True):
        line_found = bool(getattr(line_info, "found", False))
        cross_detected = bool(getattr(line_info, "cross", False))
        slow_light_active = getattr(perception, "light", None) in SLOW_LIGHTS

        if self.cooldown_until and current_time >= self.cooldown_until:
            self.cooldown_until = 0.0

        if getattr(perception, "avoid", None) is None:
            self.blocked_avoid_label = None

        if self.state == AVOID and self._avoid_elapsed(current_time):
            self._exit_avoid(current_time)

        if not camera_ok:
            self._enter(STOP, "camera_or_system_failure")
            return self.get_state_info()

        if getattr(perception, "light", None) == "red_stop":
            self._enter(STOP, "red_stop_detected")
            return self.get_state_info()

        if getattr(perception, "sign", None) == "3_Prohibited":
            self._enter(STOP, "prohibited_sign_detected")
            return self.get_state_info()

        if self.state in TURN_STATES:
            self.last_reason = "turn_maneuver_in_progress"
            return self.get_state_info()

        if self._line_lost_too_long(line_found, current_time):
            self._enter(LOST_LINE, "line_lost_timeout")
            return self.get_state_info()

        if self.state == LOST_LINE:
            self.last_reason = "lost_line_stopped"
            return self.get_state_info()

        if self.state == AVOID:
            self.last_reason = "{}_temporary_override".format(self.avoid_label)
            return self.get_state_info()

        avoid_label = getattr(perception, "avoid", None)
        if self._can_enter_avoid(avoid_label):
            self._enter_avoid(avoid_label, current_time)
            return self.get_state_info()

        if self.state == SLOW_DOWN:
            if slow_light_active:
                self.last_reason = "{}_light_slow".format(getattr(perception, "light", None))
                return self.get_state_info()
            self._exit_slow_down()

        if slow_light_active and self.state not in TURN_STATES and self.state != STOP:
            self._enter_slow_down(getattr(perception, "light", None))
            return self.get_state_info()

        if self.state == SIGN_PENDING:
            return self._update_sign_pending(cross_detected, current_time)

        if self.state == RECOVER:
            if line_found:
                self._enter(FOLLOW_LINE, "recover_line_reacquired")
            else:
                self.last_reason = "recover_searching_for_line"
            return self.get_state_info()

        if self.state == STOP:
            if line_found:
                self._enter(FOLLOW_LINE, "stop_condition_cleared")
            else:
                self.last_reason = "stop_waiting_for_line"
            return self.get_state_info()

        if self._can_accept_sign(current_time):
            sign = getattr(perception, "sign", None)
            if sign in SIGN_ACTIONS:
                self.pending_action = SIGN_ACTIONS[sign]
                self.sign_pending_start_time = current_time
                self._enter(SIGN_PENDING, self._sign_reason(sign))
                return self.get_state_info()

        self._enter(FOLLOW_LINE, "follow_line")
        return self.get_state_info()

    def notify_maneuver_complete(self, current_time):
        if self.state not in TURN_STATES:
            return
        self.pending_action = None
        self.sign_pending_start_time = None
        self.recover_start_time = current_time
        self.lost_line_since = None
        cooldown = float(self.config.get("turn.cooldown_sec", 1.5))
        self.cooldown_until = current_time + cooldown
        self._enter(RECOVER, "maneuver_complete_enter_recover")

    def get_state_info(self):
        return StateInfo(
            state=self.state,
            saved_state=self._public_saved_state(),
            pending_action=self.pending_action,
            reason=self.last_reason,
            internal_phase="NONE",
            avoid_label=self.avoid_label,
            turn_action=self._turn_action(),
            cooldown_active=self.cooldown_until > 0.0,
        )

    def _update_sign_pending(self, cross_detected, current_time):
        if cross_detected and self.pending_action:
            if self.pending_action == "LEFT":
                self._enter(TURNING_LEFT, "cross_detected_start_left_turn")
            elif self.pending_action == "RIGHT":
                self._enter(TURNING_RIGHT, "cross_detected_start_right_turn")
            else:
                self._enter(GO_STRAIGHT, "cross_detected_go_straight")
            return self.get_state_info()

        timeout = float(self.config.get("sign.pending_timeout_sec", 5.0))
        if (
            self.sign_pending_start_time is not None
            and current_time - self.sign_pending_start_time > timeout
        ):
            self.pending_action = None
            self.sign_pending_start_time = None
            self._enter(FOLLOW_LINE, "sign_pending_timeout_cleared")
            return self.get_state_info()

        self.last_reason = "waiting_for_cross"
        return self.get_state_info()

    def _line_lost_too_long(self, line_found, current_time):
        if line_found:
            self.lost_line_since = None
            return False

        if self.state in TURN_STATES:
            return False

        if self.lost_line_since is None:
            self.lost_line_since = current_time
            return False

        timeout = float(self.config.get("line.lost_timeout_sec", 0.5))
        return current_time - self.lost_line_since > timeout

    def _can_enter_avoid(self, avoid_label):
        if not avoid_label:
            return False
        if self.state in TURN_STATES or self.state == STOP:
            return False
        return avoid_label != self.blocked_avoid_label

    def _enter_avoid(self, avoid_label, current_time):
        if self.state == SLOW_DOWN:
            self.avoid_saved_state = self.slow_saved_state or FOLLOW_LINE
            self.slow_saved_state = None
        else:
            self.avoid_saved_state = self.state
        self.avoid_label = avoid_label
        self.avoid_start_time = current_time
        self._enter(AVOID, "{}_temporary_override".format(avoid_label))

    def _avoid_elapsed(self, current_time):
        if self.avoid_start_time is None:
            return False
        duration = float(self.config.get("avoid.duration_sec", 0.5))
        return current_time - self.avoid_start_time >= duration

    def _exit_avoid(self, current_time):
        restored = self.avoid_saved_state or FOLLOW_LINE
        self.blocked_avoid_label = self.avoid_label
        self.avoid_saved_state = None
        self.avoid_start_time = None
        self.avoid_label = None
        self._enter(restored, "avoid_timer_elapsed_restore_{}".format(restored.lower()))

    def _enter_slow_down(self, light_label):
        self.slow_saved_state = self.state
        self._enter(SLOW_DOWN, "{}_light_slow".format(light_label))

    def _exit_slow_down(self):
        restored = self.slow_saved_state or FOLLOW_LINE
        self.slow_saved_state = None
        self._enter(restored, "slow_condition_cleared_restore_{}".format(restored.lower()))

    def _can_accept_sign(self, current_time):
        return self.state == FOLLOW_LINE and current_time >= self.cooldown_until

    def _public_saved_state(self):
        if self.state == AVOID:
            return self.avoid_saved_state
        if self.state == SLOW_DOWN:
            return self.slow_saved_state
        return None

    def _turn_action(self):
        if self.state == TURNING_LEFT:
            return "LEFT"
        if self.state == TURNING_RIGHT:
            return "RIGHT"
        if self.state == GO_STRAIGHT:
            return "STRAIGHT"
        return None

    def _enter(self, new_state, reason):
        self.state = new_state
        self.last_reason = reason

    def _sign_reason(self, sign):
        if sign == "1_Turn_left":
            return "left_sign_confirmed"
        if sign == "2_Turn_right":
            return "right_sign_confirmed"
        return "go_straight_sign_confirmed"
