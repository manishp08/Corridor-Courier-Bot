"""Milestone 3: odometry drift on a 20 m square loop, with and without the IMU.

Each seed generates one true trajectory and one stream of wheel/gyro
measurements; every estimator consumes the identical stream, so the
comparison is paired."""
import numpy as np

from .geometry import wrap
from .localization import EKF_TUNED, EKF_UNTUNED, Ekf, WheelOdometry
from .planning import RobotLimits
from .sensors import Gyro, ImuParams, WheelEncoders

DT = 0.02   # 50 Hz, the rate of the diff-drive plugin and EKF

ESTIMATORS = {
    "wheel_only": lambda: (WheelOdometry(), False),
    "ekf_wheel_imu_tuned": lambda: (Ekf(EKF_TUNED), True),
    "ekf_wheel_imu_untuned": lambda: (Ekf(EKF_UNTUNED), True),
}


def square_commands(side=5.0, v=0.4, w=0.6):
    cmds = []
    for _ in range(4):
        cmds += [(v, 0.0)] * int(round(side / v / DT))
        cmds += [(0.0, w)] * int(round((np.pi / 2) / w / DT))
    cmds += [(0.0, 0.0)] * int(round(1.0 / DT))
    return cmds


def run_drift(seed, uncalibrated_gyro=False):
    ss = np.random.SeedSequence([seed, 3])
    r_act, r_sens = (np.random.default_rng(s) for s in ss.spawn(2))
    lim = RobotLimits()
    enc = WheelEncoders(r_sens)
    gyro = Gyro(r_sens, ImuParams(bias_sigma=0.01) if uncalibrated_gyro else ImuParams())
    pose = np.zeros(3)
    v = w = 0.0
    truth, meas = [pose.copy()], []
    dist = 0.0
    for vc, wc in square_commands():
        v += np.clip(vc - v, -lim.acc_v * DT, lim.acc_v * DT)
        w += np.clip(wc - w, -lim.acc_w * DT, lim.acc_w * DT)
        vt = v * (1 + r_act.normal(0, 0.02))
        wt = w * (1 + r_act.normal(0, 0.02))
        th = pose[2] + 0.5 * wt * DT
        pose = np.array([pose[0] + vt * np.cos(th) * DT, pose[1] + vt * np.sin(th) * DT,
                         wrap(pose[2] + wt * DT)])
        dist += abs(vt) * DT
        truth.append(pose.copy())
        meas.append((*enc.measure(vt, wt, DT, r_sens), gyro.measure(wt, DT, r_sens)))
    truth = np.array(truth)
    rows = []
    for name, make in ESTIMATORS.items():
        est, use_imu = make()
        traj = [np.zeros(3)]
        for vm, wm, gm in meas:
            traj.append(est.update(vm, wm, DT, gm if use_imu else None).copy())
        traj = np.array(traj)
        err = np.hypot(traj[:, 0] - truth[:, 0], traj[:, 1] - truth[:, 1])
        rows.append({
            "seed": seed, "estimator": name, "gyro": "uncalibrated" if uncalibrated_gyro else "calibrated",
            "distance_m": round(dist, 3),
            "final_pos_error_m": round(float(err[-1]), 4),
            "max_pos_error_m": round(float(err.max()), 4),
            "final_yaw_error_deg": round(float(np.degrees(abs(wrap(traj[-1, 2] - truth[-1, 2])))), 3),
            "drift_percent": round(100 * float(err[-1]) / dist, 3),
            "_traj": traj, "_truth": truth,
        })
    return rows
