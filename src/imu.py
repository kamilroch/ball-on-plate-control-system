import math
import threading
import time
from collections import deque

import numpy as np

from utils import angle_diff

try:
    import smbus
except ImportError:
    import smbus2 as smbus


DIR_X = -1.0
DIR_Y = +1.0

IMU_AXIS_X = np.array(
    [+0.9854, -0.1704],
    dtype=float,
)
IMU_AXIS_X /= np.linalg.norm(IMU_AXIS_X)

IMU_AXIS_Y = np.array(
    [+0.0176, -0.9998],
    dtype=float,
)
IMU_AXIS_Y /= np.linalg.norm(IMU_AXIS_Y)


class IMUThread(threading.Thread):
    def __init__(self, bus_num=1, address=0x68):
        super().__init__()

        self.bus_num = bus_num
        self.address = address
        self.bus = None

        self.base_pitch = 0.0
        self.base_roll = 0.0
        self.pitch_rel = 0.0
        self.roll_rel = 0.0
        self.alpha_x_deg = 0.0
        self.alpha_y_deg = 0.0

        self.imu_ok = False
        self.running = True

        self.lock = threading.Lock()
        self.daemon = True

        self._init_imu()

    @staticmethod
    def _word_2c(high, low):
        value = (high << 8) | low
        return value - 65536 if value >= 32768 else value

    def _init_imu(self):
        try:
            self.bus = smbus.SMBus(self.bus_num)
            self.bus.write_byte_data(
                self.address,
                0x6B,
                0x00,
            )
            time.sleep(0.15)
            self.imu_ok = True
        except Exception:
            self.imu_ok = False

    def run(self):
        pitch_queue = deque(maxlen=7)
        roll_queue = deque(maxlen=7)

        initialized = False
        alpha_filter = 0.08

        while self.running:
            if not self.imu_ok:
                time.sleep(0.5)
                self._init_imu()
                continue

            try:
                data = self.bus.read_i2c_block_data(
                    self.address,
                    0x3B,
                    14,
                )

                ax, ay, az = [
                    self._word_2c(data[i], data[i + 1]) / 16384.0
                    for i in (0, 2, 4)
                ]

                roll_raw = math.degrees(
                    math.atan2(ay, az)
                )
                pitch_raw = math.degrees(
                    math.atan2(
                        -ax,
                        math.sqrt(ay * ay + az * az),
                    )
                )

                pitch_queue.append(pitch_raw)
                roll_queue.append(roll_raw)

                if len(pitch_queue) == pitch_queue.maxlen:
                    pitch_med = float(
                        np.median(pitch_queue)
                    )

                    roll_anchor = roll_queue[0]
                    roll_med = float(
                        np.median(
                            [
                                roll_anchor
                                + angle_diff(value, roll_anchor)
                                for value in roll_queue
                            ]
                        )
                    )

                    with self.lock:
                        if not initialized:
                            self.base_pitch = pitch_med
                            self.base_roll = roll_med

                            self.pitch_rel = 0.0
                            self.roll_rel = 0.0
                            self.alpha_x_deg = 0.0
                            self.alpha_y_deg = 0.0

                            initialized = True
                        else:
                            self.pitch_rel = (
                                alpha_filter
                                * angle_diff(
                                    pitch_med,
                                    self.base_pitch,
                                )
                                + (1.0 - alpha_filter)
                                * self.pitch_rel
                            )

                            self.roll_rel = (
                                alpha_filter
                                * angle_diff(
                                    roll_med,
                                    self.base_roll,
                                )
                                + (1.0 - alpha_filter)
                                * self.roll_rel
                            )

                            imu_vector = np.array(
                                [
                                    self.pitch_rel,
                                    self.roll_rel,
                                ],
                                dtype=float,
                            )

                            self.alpha_x_deg = (
                                DIR_X
                                * float(IMU_AXIS_X @ imu_vector)
                            )
                            self.alpha_y_deg = (
                                DIR_Y
                                * float(IMU_AXIS_Y @ imu_vector)
                            )

                        self.imu_ok = True

            except Exception:
                with self.lock:
                    self.imu_ok = False

            time.sleep(0.01)
