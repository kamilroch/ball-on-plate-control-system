import math

import numpy as np
from scipy.linalg import solve_continuous_are

from utils import clamp, vector_limit


G = 9.81
K_BALL = (3.0 / 5.0) * G

TS_X = 0.1325
KS_X = 0.1372
TS_Y = 0.1294
KS_Y = 0.1320

A_X = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, K_BALL],
        [0.0, 0.0, -1.0 / TS_X],
    ],
    dtype=float,
)
B_X = np.array(
    [
        [0.0],
        [0.0],
        [KS_X / TS_X],
    ],
    dtype=float,
)

A_Y = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, K_BALL],
        [0.0, 0.0, -1.0 / TS_Y],
    ],
    dtype=float,
)
B_Y = np.array(
    [
        [0.0],
        [0.0],
        [KS_Y / TS_Y],
    ],
    dtype=float,
)

Q_X = np.diag([12000.0, 2500.0, 1.0])
Q_Y = np.diag([20000.0, 2500.0, 1.0])

R_X = np.array([[150.0]])
R_Y = np.array([[150.0]])

P_X = solve_continuous_are(A_X, B_X, Q_X, R_X)
K_X_MAT = np.linalg.inv(R_X) @ B_X.T @ P_X

P_Y = solve_continuous_are(A_Y, B_Y, Q_Y, R_Y)
K_Y_MAT = np.linalg.inv(R_Y) @ B_Y.T @ P_Y

MAX_PLATFORM_TILT_DEG = 9.0
MAX_U_X_DEG = 70.0
MAX_U_Y_DEG = 42.0

BASE_TILT_COMP_ENABLED = True
BASE_TILT_FILTER_TAU_S = 1.20
BASE_TILT_DEADBAND_DEG = 0.12
BASE_TILT_COMP_GAIN = 0.30
BASE_TILT_COMP_MAX_U_DEG = 6.0
BASE_TILT_EST_MAX_DEG = 4.0


def first_order_exact(alpha_rad, u_rad, ts, ks, dt):
    decay = math.exp(-dt / ts)
    return decay * alpha_rad + ks * (1.0 - decay) * u_rad


def limit_model_input(u_x_deg, u_y_deg):
    alpha_ss_x_deg = KS_X * u_x_deg
    alpha_ss_y_deg = KS_Y * u_y_deg

    alpha_ss_x_deg, alpha_ss_y_deg = vector_limit(
        alpha_ss_x_deg,
        alpha_ss_y_deg,
        MAX_PLATFORM_TILT_DEG,
    )

    u_x_deg = alpha_ss_x_deg / KS_X
    u_y_deg = alpha_ss_y_deg / KS_Y

    return (
        clamp(u_x_deg, -MAX_U_X_DEG, MAX_U_X_DEG),
        clamp(u_y_deg, -MAX_U_Y_DEG, MAX_U_Y_DEG),
    )


def calculate_lqr_control(
    x_m,
    y_m,
    vx,
    vy,
    alpha_hat_x_rad,
    alpha_hat_y_rad,
    velocity_limit_mps,
):
    vx_lqr = clamp(vx, -velocity_limit_mps, velocity_limit_mps)
    vy_lqr = clamp(vy, -velocity_limit_mps, velocity_limit_mps)

    state_x = np.array(
        [[x_m], [vx_lqr], [alpha_hat_x_rad]],
        dtype=float,
    )
    state_y = np.array(
        [[y_m], [vy_lqr], [alpha_hat_y_rad]],
        dtype=float,
    )

    u_x_rad = -float((K_X_MAT @ state_x)[0, 0])
    u_y_rad = -float((K_Y_MAT @ state_y)[0, 0])

    return math.degrees(u_x_rad), math.degrees(u_y_rad)
