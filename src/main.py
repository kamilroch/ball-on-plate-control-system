import csv
import math
import time
from collections import deque

from control import (
    BASE_TILT_COMP_ENABLED,
    BASE_TILT_COMP_GAIN,
    BASE_TILT_COMP_MAX_U_DEG,
    BASE_TILT_DEADBAND_DEG,
    BASE_TILT_EST_MAX_DEG,
    BASE_TILT_FILTER_TAU_S,
    KS_X,
    KS_Y,
    TS_X,
    TS_Y,
    calculate_lqr_control,
    first_order_exact,
    limit_model_input,
)
from hardware import ServoController
from imu import IMUThread
from utils import clamp, soft_deadzone, update_velocity_estimate
from vision import (
    CENTER_X,
    CENTER_Y,
    create_camera,
    detect_ball_hsv94,
)


ALPHA_POS = 0.60

VEL_FAST_WINDOW = 5
VEL_FAST_FILTER_ALPHA = 0.45
LQR_VELOCITY_LIMIT_MPS = 0.025

PLATE_SIZE = 0.25
PX_TO_M_X = PLATE_SIZE / (820 - 175)
PX_TO_M_Y = PLATE_SIZE / (640 - 0)

LOG_NAME = "logi.csv"


def main():
    servo = ServoController()

    imu_reader = IMUThread()
    imu_reader.start()
    time.sleep(0.3)

    camera = create_camera()

    smooth_x = None
    smooth_y = None

    x_dot_f = 0.0
    y_dot_f = 0.0

    vel_samples_x = deque(
        maxlen=VEL_FAST_WINDOW
    )
    vel_samples_y = deque(
        maxlen=VEL_FAST_WINDOW
    )

    alpha_hat_x_rad = 0.0
    alpha_hat_y_rad = 0.0

    base_tilt_est_x_deg = 0.0
    base_tilt_est_y_deg = 0.0

    tracking_state = 0
    lost_frames = 0
    tracked_radius_px = None

    prev_loop_time = None
    last_measure_time = None

    fps_time = time.time()
    frame_count = 0
    fps_count = 0

    log_file = open(
        LOG_NAME,
        "w",
        newline="",
        buffering=1,
    )

    log_writer = csv.writer(log_file)
    log_writer.writerow(
        [
            "t",
            "frame",
            "dt_s",
            "x_px",
            "y_px",
            "radius_px",
            "vision_source",
            "x_m",
            "y_m",
            "vx_mps",
            "vy_mps",
            "alpha_hat_x_deg",
            "alpha_hat_y_deg",
            "alpha_imu_x_deg",
            "alpha_imu_y_deg",
            "imu_ok",
            "base_tilt_est_x_deg",
            "base_tilt_est_y_deg",
            "u_lqr_x_deg",
            "u_lqr_y_deg",
            "u_comp_x_deg",
            "u_comp_y_deg",
            "u_cmd_x_deg",
            "u_cmd_y_deg",
            "u_applied_x_deg",
            "u_applied_y_deg",
            "sX",
            "sY",
            "ball_detected",
            "mode",
        ]
    )

    print(
        "\nSTART - LQR + HSV94 + IMU base tilt compensation"
    )
    print(f"CSV log: {LOG_NAME}")

    try:
        while True:
            frame = camera.capture_array()
            now = time.perf_counter()

            frame_count += 1
            fps_count += 1

            dt = (
                0.033
                if prev_loop_time is None
                else clamp(
                    now - prev_loop_time,
                    0.015,
                    0.10,
                )
            )
            prev_loop_time = now

            with imu_reader.lock:
                imu_ax_deg = float(
                    imu_reader.alpha_x_deg
                )
                imu_ay_deg = float(
                    imu_reader.alpha_y_deg
                )
                imu_ok = bool(
                    imu_reader.imu_ok
                )

            if BASE_TILT_COMP_ENABLED and imu_ok:
                model_ax_deg = math.degrees(
                    alpha_hat_x_rad
                )
                model_ay_deg = math.degrees(
                    alpha_hat_y_rad
                )

                residual_x_deg = clamp(
                    imu_ax_deg - model_ax_deg,
                    -BASE_TILT_EST_MAX_DEG,
                    BASE_TILT_EST_MAX_DEG,
                )
                residual_y_deg = clamp(
                    imu_ay_deg - model_ay_deg,
                    -BASE_TILT_EST_MAX_DEG,
                    BASE_TILT_EST_MAX_DEG,
                )

                imu_blend = clamp(
                    dt
                    / (
                        BASE_TILT_FILTER_TAU_S
                        + dt
                    ),
                    0.0,
                    1.0,
                )

                base_tilt_est_x_deg += (
                    imu_blend
                    * (
                        residual_x_deg
                        - base_tilt_est_x_deg
                    )
                )
                base_tilt_est_y_deg += (
                    imu_blend
                    * (
                        residual_y_deg
                        - base_tilt_est_y_deg
                    )
                )

            vision_candidate, _ = (
                detect_ball_hsv94(frame)
            )

            best = None
            vision_source = "NONE"

            if vision_candidate is not None:
                best = (
                    int(
                        round(
                            vision_candidate["x"]
                        )
                    ),
                    int(
                        round(
                            vision_candidate["y"]
                        )
                    ),
                    int(
                        round(
                            vision_candidate["r"]
                        )
                    ),
                )

                vision_source = (
                    "HSV94_EDGE"
                    if vision_candidate["edge"]
                    else "HSV94"
                )

            if best is not None:
                log_x_px, log_y_px, log_r_px = (
                    best
                )
            else:
                log_x_px = float("nan")
                log_y_px = float("nan")
                log_r_px = float("nan")

            display_x = 0.0
            display_y = 0.0
            display_vx = 0.0
            display_vy = 0.0

            u_cmd_x_deg = 0.0
            u_cmd_y_deg = 0.0

            u_lqr_x_deg = 0.0
            u_lqr_y_deg = 0.0

            u_comp_x_deg = 0.0
            u_comp_y_deg = 0.0

            if best is not None:
                gap_frames = lost_frames
                lost_frames = 0

                x_px, y_px, r_px = best

                if (
                    tracking_state != 2
                    or smooth_x is None
                    or smooth_y is None
                ):
                    tracking_state = 2

                    smooth_x = float(x_px)
                    smooth_y = float(y_px)
                    tracked_radius_px = float(r_px)

                    x_m = (
                        -(smooth_x - CENTER_X)
                        * PX_TO_M_X
                    )
                    y_m = (
                        -(smooth_y - CENTER_Y)
                        * PX_TO_M_Y
                    )

                    last_measure_time = now

                    x_dot_f = 0.0
                    y_dot_f = 0.0
                    vx = 0.0
                    vy = 0.0

                    vel_samples_x.clear()
                    vel_samples_y.clear()

                    vel_samples_x.append(
                        (now, x_m)
                    )
                    vel_samples_y.append(
                        (now, y_m)
                    )

                else:
                    if tracked_radius_px is None:
                        tracked_radius_px = float(
                            r_px
                        )
                    else:
                        tracked_radius_px = (
                            0.80 * tracked_radius_px
                            + 0.20 * float(r_px)
                        )

                    smooth_x = (
                        ALPHA_POS * float(x_px)
                        + (1.0 - ALPHA_POS)
                        * smooth_x
                    )
                    smooth_y = (
                        ALPHA_POS * float(y_px)
                        + (1.0 - ALPHA_POS)
                        * smooth_y
                    )

                    x_m = (
                        -(smooth_x - CENTER_X)
                        * PX_TO_M_X
                    )
                    y_m = (
                        -(smooth_y - CENTER_Y)
                        * PX_TO_M_Y
                    )

                    meas_dt = (
                        dt
                        if last_measure_time is None
                        else clamp(
                            now - last_measure_time,
                            0.015,
                            0.20,
                        )
                    )
                    last_measure_time = now

                    if gap_frames >= 2:
                        vel_samples_x.clear()
                        vel_samples_y.clear()

                        x_dot_f = 0.0
                        y_dot_f = 0.0

                        vel_samples_x.append(
                            (now, x_m)
                        )
                        vel_samples_y.append(
                            (now, y_m)
                        )

                    else:
                        x_dot_f = (
                            update_velocity_estimate(
                                vel_samples_x,
                                now,
                                x_m,
                                x_dot_f,
                                meas_dt,
                                VEL_FAST_FILTER_ALPHA,
                            )
                        )
                        y_dot_f = (
                            update_velocity_estimate(
                                vel_samples_y,
                                now,
                                y_m,
                                y_dot_f,
                                meas_dt,
                                VEL_FAST_FILTER_ALPHA,
                            )
                        )

                    vx = x_dot_f
                    vy = y_dot_f

                display_x = x_m
                display_y = y_m
                display_vx = vx
                display_vy = vy

                (
                    u_lqr_x_deg,
                    u_lqr_y_deg,
                ) = calculate_lqr_control(
                    x_m,
                    y_m,
                    vx,
                    vy,
                    alpha_hat_x_rad,
                    alpha_hat_y_rad,
                    LQR_VELOCITY_LIMIT_MPS,
                )

                if (
                    BASE_TILT_COMP_ENABLED
                    and imu_ok
                ):
                    tilt_x_eff = soft_deadzone(
                        base_tilt_est_x_deg,
                        BASE_TILT_DEADBAND_DEG,
                    )
                    tilt_y_eff = soft_deadzone(
                        base_tilt_est_y_deg,
                        BASE_TILT_DEADBAND_DEG,
                    )

                    u_comp_x_deg = clamp(
                        -BASE_TILT_COMP_GAIN
                        * tilt_x_eff
                        / max(KS_X, 1e-9),
                        -BASE_TILT_COMP_MAX_U_DEG,
                        BASE_TILT_COMP_MAX_U_DEG,
                    )
                    u_comp_y_deg = clamp(
                        -BASE_TILT_COMP_GAIN
                        * tilt_y_eff
                        / max(KS_Y, 1e-9),
                        -BASE_TILT_COMP_MAX_U_DEG,
                        BASE_TILT_COMP_MAX_U_DEG,
                    )

                u_cmd_x_deg = (
                    u_lqr_x_deg
                    + u_comp_x_deg
                )
                u_cmd_y_deg = (
                    u_lqr_y_deg
                    + u_comp_y_deg
                )

                (
                    u_cmd_x_deg,
                    u_cmd_y_deg,
                ) = limit_model_input(
                    u_cmd_x_deg,
                    u_cmd_y_deg,
                )

                mode = "LQR+IMU_COMP"

            else:
                if tracking_state == 2:
                    lost_frames += 1

                    if lost_frames > 3:
                        tracking_state = 0
                        tracked_radius_px = None
                        smooth_x = None
                        smooth_y = None

                        vel_samples_x.clear()
                        vel_samples_y.clear()

                        x_dot_f = 0.0
                        y_dot_f = 0.0

                u_cmd_x_deg = 0.0
                u_cmd_y_deg = 0.0

                mode = (
                    "COAST"
                    if tracking_state == 2
                    else "NO_BALL"
                )

            if tracking_state != 2:
                u_cmd_x_deg = 0.0
                u_cmd_y_deg = 0.0

            (
                angle_x,
                angle_y,
                u_applied_x_deg,
                u_applied_y_deg,
            ) = servo.apply_control(
                u_cmd_x_deg,
                u_cmd_y_deg,
                dt,
            )

            u_applied_x_rad = math.radians(
                u_applied_x_deg
            )
            u_applied_y_rad = math.radians(
                u_applied_y_deg
            )

            alpha_hat_x_rad = first_order_exact(
                alpha_hat_x_rad,
                u_applied_x_rad,
                TS_X,
                KS_X,
                dt,
            )
            alpha_hat_y_rad = first_order_exact(
                alpha_hat_y_rad,
                u_applied_y_rad,
                TS_Y,
                KS_Y,
                dt,
            )

            log_writer.writerow(
                [
                    time.time(),
                    frame_count,
                    f"{dt:.6f}",
                    log_x_px,
                    log_y_px,
                    log_r_px,
                    vision_source,
                    f"{display_x:.6f}",
                    f"{display_y:.6f}",
                    f"{display_vx:.6f}",
                    f"{display_vy:.6f}",
                    f"{math.degrees(alpha_hat_x_rad):.6f}",
                    f"{math.degrees(alpha_hat_y_rad):.6f}",
                    f"{imu_ax_deg:.6f}",
                    f"{imu_ay_deg:.6f}",
                    int(imu_ok),
                    f"{base_tilt_est_x_deg:.6f}",
                    f"{base_tilt_est_y_deg:.6f}",
                    f"{u_lqr_x_deg:.6f}",
                    f"{u_lqr_y_deg:.6f}",
                    f"{u_comp_x_deg:.6f}",
                    f"{u_comp_y_deg:.6f}",
                    f"{u_cmd_x_deg:.6f}",
                    f"{u_cmd_y_deg:.6f}",
                    f"{u_applied_x_deg:.6f}",
                    f"{u_applied_y_deg:.6f}",
                    f"{angle_x:.3f}",
                    f"{angle_y:.3f}",
                    int(best is not None),
                    mode,
                ]
            )

            now_fps = time.time()

            if now_fps - fps_time >= 0.5:
                fps = (
                    fps_count
                    / (now_fps - fps_time)
                )

                if tracking_state == 2:
                    print(
                        f"FPS:{fps:4.1f} "
                        f"dt:{dt * 1000:4.0f}ms "
                        f"SRC:{vision_source:8s} | "
                        f"X:{display_x:+.3f} "
                        f"Vx:{display_vx:+.3f} "
                        f"sX:{angle_x:5.1f} | "
                        f"Y:{display_y:+.3f} "
                        f"Vy:{display_vy:+.3f} "
                        f"sY:{angle_y:5.1f} | "
                        f"BT=("
                        f"{base_tilt_est_x_deg:+.2f},"
                        f"{base_tilt_est_y_deg:+.2f}) "
                        f"C=("
                        f"{u_comp_x_deg:+.1f},"
                        f"{u_comp_y_deg:+.1f}) | "
                        f"{mode}"
                    )
                else:
                    print(
                        f"FPS:{fps:4.1f} | "
                        f"NO BALL | "
                        f"aX:"
                        f"{math.degrees(alpha_hat_x_rad):+.2f}° "
                        f"aY:"
                        f"{math.degrees(alpha_hat_y_rad):+.2f}°"
                    )

                fps_count = 0
                fps_time = now_fps

    except KeyboardInterrupt:
        print("\nStopped.")

    finally:
        imu_reader.running = False
        servo.reset()

        try:
            camera.stop()
        except Exception:
            pass

        try:
            log_file.close()
        except Exception:
            pass

        print("Finished.")


if __name__ == "__main__":
    main()
