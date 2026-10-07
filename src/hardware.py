import time

import numpy as np
from adafruit_servokit import ServoKit

from control import KS_X, KS_Y
from utils import clamp


SERVO_Y = 0
SERVO_X = 3

SERVO_X_NEUTRAL_BASE = 95.0
SERVO_Y_NEUTRAL_BASE = 78.0

SERVO_X_MIN = 20.0
SERVO_X_MAX = 145.0
SERVO_Y_MIN = 15.0
SERVO_Y_MAX = 140.0

MAX_SERVO_SPEED_X = 300.0
MAX_SERVO_SPEED_Y = 300.0
SERVO_SLEW_DT_MAX = 0.040

SERVO_TO_IMU = np.array(
    [
        [0.14796497, 0.01493174],
        [-0.00131039, -0.13770057],
    ],
    dtype=float,
)

IMU_TO_SERVO = np.linalg.inv(SERVO_TO_IMU)
DECOUPLER = IMU_TO_SERVO @ np.diag([-KS_X, -KS_Y])


class ServoController:
    def __init__(self):
        self.kit = ServoKit(channels=16)

        self.neutral_x = float(SERVO_X_NEUTRAL_BASE)
        self.neutral_y = float(SERVO_Y_NEUTRAL_BASE)

        self.last_angle_x = self.neutral_x
        self.last_angle_y = self.neutral_y

        self.kit.servo[SERVO_X].angle = self.neutral_x
        self.kit.servo[SERVO_Y].angle = self.neutral_y
        time.sleep(1.0)

    def virtual_u_to_servo_delta(self, u_x_deg, u_y_deg):
        servo_delta = DECOUPLER @ np.array(
            [u_x_deg, u_y_deg],
            dtype=float,
        )
        return float(servo_delta[0]), float(servo_delta[1])

    @staticmethod
    def servo_delta_to_virtual_u(delta_servo_x_deg, delta_servo_y_deg):
        imu_vector = SERVO_TO_IMU @ np.array(
            [delta_servo_x_deg, delta_servo_y_deg],
            dtype=float,
        )

        return (
            -float(imu_vector[0]) / max(KS_X, 1e-9),
            -float(imu_vector[1]) / max(KS_Y, 1e-9),
        )

    def apply_control(self, u_cmd_x_deg, u_cmd_y_deg, dt):
        servo_delta_cmd_x, servo_delta_cmd_y = (
            self.virtual_u_to_servo_delta(
                u_cmd_x_deg,
                u_cmd_y_deg,
            )
        )

        target_servo_x = clamp(
            self.neutral_x + servo_delta_cmd_x,
            SERVO_X_MIN,
            SERVO_X_MAX,
        )
        target_servo_y = clamp(
            self.neutral_y + servo_delta_cmd_y,
            SERVO_Y_MIN,
            SERVO_Y_MAX,
        )

        servo_slew_dt = min(dt, SERVO_SLEW_DT_MAX)
        max_step_x = MAX_SERVO_SPEED_X * servo_slew_dt
        max_step_y = MAX_SERVO_SPEED_Y * servo_slew_dt

        delta_x = clamp(
            target_servo_x - self.last_angle_x,
            -max_step_x,
            max_step_x,
        )
        delta_y = clamp(
            target_servo_y - self.last_angle_y,
            -max_step_y,
            max_step_y,
        )

        angle_x = clamp(
            self.last_angle_x + delta_x,
            SERVO_X_MIN,
            SERVO_X_MAX,
        )
        angle_y = clamp(
            self.last_angle_y + delta_y,
            SERVO_Y_MIN,
            SERVO_Y_MAX,
        )

        self.kit.servo[SERVO_X].angle = angle_x
        self.kit.servo[SERVO_Y].angle = angle_y

        self.last_angle_x = angle_x
        self.last_angle_y = angle_y

        u_applied_x_deg, u_applied_y_deg = (
            self.servo_delta_to_virtual_u(
                angle_x - self.neutral_x,
                angle_y - self.neutral_y,
            )
        )

        return (
            angle_x,
            angle_y,
            u_applied_x_deg,
            u_applied_y_deg,
        )

    def reset(self):
        try:
            self.kit.servo[SERVO_X].angle = self.neutral_x
            self.kit.servo[SERVO_Y].angle = self.neutral_y
            time.sleep(0.5)
        except Exception:
            pass
