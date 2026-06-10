from dataclasses import dataclass


APPROACH = "APPROACH"
TURN = "TURN"
GO_STRAIGHT_PHASE = "GO_STRAIGHT"
COMPLETE = "COMPLETE"
NONE = "NONE"


@dataclass
class TurnOutput:
    steering: float
    throttle: float
    internal_phase: str
    reason: str
    complete: bool = False


class TurnController:
    def __init__(self, config):
        self.config = config
        self.active_action = None
        self.phase = NONE
        self.start_time = None
        self.phase_start_time = None

    def reset(self):
        self.active_action = None
        self.phase = NONE
        self.start_time = None
        self.phase_start_time = None

    def update(self, action, line_info, current_time):
        if action != self.active_action or self.phase == NONE:
            self._start(action, current_time)

        if self.phase == APPROACH:
            return self._update_approach(current_time)

        if self.phase == GO_STRAIGHT_PHASE:
            return self._update_go_straight(current_time)

        if self.phase == TURN:
            return self._update_turn(line_info, current_time)

        return TurnOutput(0.0, 0.0, COMPLETE, "turn_complete", complete=True)

    def _start(self, action, current_time):
        self.active_action = action
        self.phase = APPROACH
        self.start_time = current_time
        self.phase_start_time = current_time

    def _update_approach(self, current_time):
        approach_time = float(self.config.get("turn.approach_time_sec", 0.25))
        if current_time - self.phase_start_time < approach_time:
            return TurnOutput(
                steering=float(self.config.get("steering.straight", 0.0)),
                throttle=float(self.config.get("throttle.turn", 0.09)),
                internal_phase=APPROACH,
                reason="turn_internal_approach",
            )

        self.phase = GO_STRAIGHT_PHASE if self.active_action == "STRAIGHT" else TURN
        self.phase_start_time = current_time
        return self.update(self.active_action, None, current_time)

    def _update_go_straight(self, current_time):
        duration = float(self.config.get("turn.go_straight_time_sec", 0.6))
        if current_time - self.phase_start_time >= duration:
            self.phase = COMPLETE
            return TurnOutput(0.0, 0.0, COMPLETE, "go_straight_complete", complete=True)

        return TurnOutput(
            steering=float(self.config.get("steering.straight", 0.0)),
            throttle=float(self.config.get("throttle.turn", 0.09)),
            internal_phase=GO_STRAIGHT_PHASE,
            reason="go_straight",
        )

    def _update_turn(self, line_info, current_time):
        min_turn = float(self.config.get("turn.min_turn_time_sec", 0.5))
        max_turn = float(self.config.get("turn.max_turn_time_sec", 2.5))
        turn_elapsed = current_time - self.phase_start_time
        total_elapsed = current_time - self.start_time
        line_found = bool(getattr(line_info, "found", False))

        if (turn_elapsed >= min_turn and line_found) or total_elapsed >= max_turn:
            self.phase = COMPLETE
            reason = "turn_line_reacquired" if line_found else "turn_timeout_complete"
            return TurnOutput(0.0, 0.0, COMPLETE, reason, complete=True)

        if self.active_action == "LEFT":
            steering = float(self.config.get("steering.turn_left", 0.7))
            reason = "turn_left"
        else:
            steering = float(self.config.get("steering.turn_right", -0.7))
            reason = "turn_right"

        return TurnOutput(
            steering=steering,
            throttle=float(self.config.get("throttle.turn", 0.09)),
            internal_phase=TURN,
            reason=reason,
        )
