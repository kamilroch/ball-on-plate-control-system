import math
from collections import deque

import numpy as np


def clamp(value, low, high):
    return max(low, min(high, value))


def angle_diff(angle, reference):
    return ((angle - reference + 180.0) % 360.0) - 180.0


def vector_limit(x_value, y_value, maximum):
    magnitude = math.hypot(x_value, y_value)
    if magnitude <= maximum or magnitude <= 1e-12:
        return x_value, y_value

    scale = maximum / magnitude
    return x_value * scale, y_value * scale


def soft_deadzone(value, deadband):
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0

    return math.copysign(magnitude - deadband, value)


def regression_velocity(samples):
    if len(samples) < 3:
        return 0.0

    t0 = samples[0][0]
    t = np.array([sample[0] - t0 for sample in samples], dtype=float)
    p = np.array([sample[1] for sample in samples], dtype=float)

    t_mean = float(np.mean(t))
    p_mean = float(np.mean(p))
    dt_values = t - t_mean

    denominator = float(np.sum(dt_values * dt_values))
    if denominator <= 1e-12:
        return 0.0

    slope = float(np.sum(dt_values * (p - p_mean)) / denominator)
    return clamp(slope, -3.0, 3.0)


def update_velocity_estimate(
    samples,
    now,
    position_m,
    previous_velocity,
    dt,
    filter_alpha,
):
    samples.append((now, position_m))

    raw_velocity = regression_velocity(samples)
    velocity = (
        filter_alpha * raw_velocity
        + (1.0 - filter_alpha) * previous_velocity
    )

    max_dv = 2.0 * ((3.0 / 5.0) * 9.81) * math.radians(9.0) * dt
    velocity = clamp(
        velocity,
        previous_velocity - max_dv,
        previous_velocity + max_dv,
    )
    velocity = clamp(velocity, -3.0, 3.0)

    return soft_deadzone(velocity, 0.006)
