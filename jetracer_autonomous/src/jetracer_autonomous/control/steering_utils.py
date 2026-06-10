def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def smooth_value(target, previous, alpha):
    alpha = clamp(alpha, 0.0, 1.0)
    return alpha * previous + (1.0 - alpha) * target


def limit_delta(target, previous, max_delta):
    delta = target - previous
    if delta > max_delta:
        return previous + max_delta
    if delta < -max_delta:
        return previous - max_delta
    return target
