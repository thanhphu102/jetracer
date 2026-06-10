class DebugLogger:
    def __init__(self, config, ros_log=None):
        self.show_log = bool(config.get("debug.show_log", True))
        self.ros_log = ros_log

    def log(self, state_info, line_info, perception, command, yolo_ran):
        if not self.show_log:
            return

        line_error = self._fmt(getattr(line_info, "line_error", None))
        message = (
            "state={state} saved_state={saved_state} internal_phase={phase} "
            "line_error={line_error} cross={cross} "
            "left={left:.3f} center={center:.3f} right={right:.3f} "
            "sign={sign} light={light} avoid={avoid} pending={pending} "
            "yolo_ran={yolo_ran} steering={steering:.3f} throttle={throttle:.3f} "
            "reason={reason}"
        ).format(
            state=state_info.state,
            saved_state=state_info.saved_state,
            phase=command.internal_phase,
            line_error=line_error,
            cross=getattr(line_info, "cross", False),
            left=getattr(line_info, "left_score", 0.0),
            center=getattr(line_info, "center_score", 0.0),
            right=getattr(line_info, "right_score", 0.0),
            sign=getattr(perception, "sign", None),
            light=getattr(perception, "light", None),
            avoid=getattr(perception, "avoid", None),
            pending=state_info.pending_action,
            yolo_ran=yolo_ran,
            steering=command.steering,
            throttle=command.throttle,
            reason=command.reason or state_info.reason,
        )

        if self.ros_log:
            self.ros_log(message)
        else:
            print(message)

    def _fmt(self, value):
        if value is None:
            return "None"
        return "{:.1f}".format(value)
