#!/usr/bin/env python3
# EKF for BNO055 + Adafruit Ultimate GPS on a Raspberry Pi
# state x (9x1):
#   pE, pN, pU      position east, north, up from the first GPS fix (m)
#   vE, vN, vU      velocity east, north, up (m/s)
#   baX, baY, baZ   accelerometer bias along the IMU's x, y, z axes (m/s^2)
#
# python3 ekf.py --no-gps --zupt
# python3 ekf.py --gps-port /dev/ttyAMA0

import argparse
import csv
import math
import queue
import threading
import time

import numpy as np

# placeholder noise values -- replace after the stationary test
CONFIG = {
    "imu_rate_hz": 50,
    "sigma_accel": 0.05,        # m/s^2
    "sigma_accel_bias": 1e-4,   # m/s^2/sqrt(s)
    "gps_uere": 2.5,            # m per unit HDOP
    "gps_vert_factor": 2.0,
    "gps_vel_sigma": 0.2,       # m/s
    "gps_vel_min_speed": 0.5,
    "gps_rate_hz": 5,
    "zupt_accel_thresh": 0.15,
    "zupt_gyro_thresh": 0.03,
    "zupt_window": 15,
    "zupt_sigma": 0.03,
    "yaw_offset_deg": 0.0,      # check by pushing the board north
    "gate_3dof": 11.34,
    "gate_2dof": 9.21,
}

EARTH_R = 6378137.0


def quat_to_rot(w, x, y, z):
    n = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


def rot_z(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def lla_to_enu(lat, lon, alt, origin):
    lat0, lon0, alt0 = origin
    e = math.radians(lon - lon0) * EARTH_R * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * EARTH_R
    return np.array([e, n, alt - alt0])


class EKF:
    def __init__(self, cfg):
        self.cfg = cfg
        self.x = np.zeros(9)
        self.P = np.diag([1.0]*3 + [0.1]*3 + [0.05**2]*3)

    def predict(self, a_body, R, dt):
        v, b = self.x[3:6], self.x[6:9]
        a_w = R @ (a_body - b)

        self.x[0:3] += v*dt + 0.5*a_w*dt*dt
        self.x[3:6] += a_w*dt

        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        F[0:3, 6:9] = -0.5 * R * dt*dt
        F[3:6, 6:9] = -R * dt

        G = np.vstack([0.5*R*dt*dt, R*dt, np.zeros((3, 3))])
        Q = G @ (self.cfg["sigma_accel"]**2 * np.eye(3)) @ G.T
        Q[6:9, 6:9] += self.cfg["sigma_accel_bias"]**2 * dt * np.eye(3)

        self.P = F @ self.P @ F.T + Q

    def update(self, z, H, Rm, gate=None):
        y = z - H @ self.x
        S = H @ self.P @ H.T + Rm
        Si = np.linalg.inv(S)
        nis = float(y @ Si @ y)
        if gate is not None and nis > gate:
            return False, nis
        K = self.P @ H.T @ Si
        self.x += K @ y
        A = np.eye(9) - K @ H
        self.P = A @ self.P @ A.T + K @ Rm @ K.T
        return True, nis


def checksum(s):
    c = 0
    for ch in s:
        c ^= ord(ch)
    return f"{c:02X}"


def pmtk(ser, cmd):
    ser.write(f"${cmd}*{checksum(cmd)}\r\n".encode())
    time.sleep(0.1)


class GPSReader(threading.Thread):
    def __init__(self, port, baud, rate_hz, q):
        super().__init__(daemon=True)
        self.port, self.baud, self.rate_hz, self.q = port, baud, rate_hz, q
        self.stop = threading.Event()

    def run(self):
        import serial
        import pynmea2

        ser = serial.Serial(self.port, self.baud, timeout=1)
        pmtk(ser, "PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")  # RMC + GGA
        pmtk(ser, f"PMTK220,{int(1000 / self.rate_hz)}")

        while not self.stop.is_set():
            line = ser.readline().decode("ascii", errors="ignore").strip()
            if not line.startswith("$"):
                continue
            try:
                msg = pynmea2.parse(line)
            except pynmea2.ParseError:
                continue
            t = time.monotonic()
            if msg.sentence_type == "GGA" and msg.gps_qual and int(msg.gps_qual) > 0:
                self.q.put(("pos", t, msg.latitude, msg.longitude,
                            float(msg.altitude or 0), float(msg.horizontal_dil or 99),
                            int(msg.num_sats or 0)))
            elif msg.sentence_type == "RMC" and msg.status == "A":
                spd = float(msg.spd_over_grnd or 0) * 0.514444
                crs = float(msg.true_course) if msg.true_course else None
                self.q.put(("vel", t, spd, crs))


def valid(*vals):
    return all(v is not None and None not in v for v in vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gps", action="store_true")
    ap.add_argument("--gps-port", default="/dev/serial0")
    ap.add_argument("--gps-baud", type=int, default=9600)
    ap.add_argument("--zupt", action="store_true")
    ap.add_argument("--log", default="ekf_log.csv")
    args = ap.parse_args()

    import board
    import adafruit_bno055

    cfg = CONFIG
    imu = adafruit_bno055.BNO055_I2C(board.I2C())  # SDA = GPIO2 (pin 3), SCL = GPIO3 (pin 5)
    R_yaw = rot_z(cfg["yaw_offset_deg"])
    ekf = EKF(cfg)

    gps_q = queue.Queue()
    gps = None
    if not args.no_gps:
        gps = GPSReader(args.gps_port, args.gps_baud, cfg["gps_rate_hz"], gps_q)
        gps.start()
    origin = None
    hdop, sats = float("nan"), 0

    still = 0
    period = 1.0 / cfg["imu_rate_hz"]
    t_prev = time.monotonic()
    t_print = t_prev

    f = open(args.log, "w", newline="")
    log = csv.writer(f)
    log.writerow(["t", "dt", "pE", "pN", "pU", "vE", "vN", "vU", "baX", "baY", "baZ",
                  "sig_pE", "sig_pN", "sig_pU", "aX", "aY", "aZ", "gX", "gY", "gZ",
                  "qw", "qx", "qy", "qz", "cal_sys", "cal_gyro", "cal_acc", "cal_mag",
                  "event"])

    print("logging to", args.log)
    try:
        while True:
            t_loop = time.monotonic()
            q, a, g = imu.quaternion, imu.linear_acceleration, imu.gyro
            if not valid(q, a, g):
                continue

            t = time.monotonic()
            dt = t - t_prev
            t_prev = t
            if dt <= 0 or dt > 0.5:
                continue

            a = np.array(a, dtype=float)
            g = np.array(g, dtype=float)
            R = R_yaw @ quat_to_rot(*q)
            ekf.predict(a, R, dt)
            events = []

            if args.zupt:
                if np.linalg.norm(a) < cfg["zupt_accel_thresh"] and \
                   np.linalg.norm(g) < cfg["zupt_gyro_thresh"]:
                    still += 1
                else:
                    still = 0
                if still >= cfg["zupt_window"]:
                    H = np.zeros((3, 9))
                    H[:, 3:6] = np.eye(3)
                    ekf.update(np.zeros(3), H, cfg["zupt_sigma"]**2 * np.eye(3))
                    events.append("zupt")

            while not gps_q.empty():
                m = gps_q.get_nowait()
                if m[0] == "pos":
                    _, _, lat, lon, alt, hdop, sats = m
                    if origin is None:
                        origin = (lat, lon, alt)
                        ekf.x[0:3] = 0
                        ekf.P[0:3, 0:3] = np.eye(3) * (hdop * cfg["gps_uere"])**2
                        events.append("origin")
                        continue
                    z = lla_to_enu(lat, lon, alt, origin)
                    sh = hdop * cfg["gps_uere"]
                    sv = sh * cfg["gps_vert_factor"]
                    H = np.zeros((3, 9))
                    H[:, 0:3] = np.eye(3)
                    ok, nis = ekf.update(z, H, np.diag([sh**2, sh**2, sv**2]), cfg["gate_3dof"])
                    events.append(f"pos{'' if ok else '_rej'}:{nis:.1f}")
                else:
                    _, _, spd, crs = m
                    if crs is None or spd < cfg["gps_vel_min_speed"]:
                        continue
                    c = math.radians(crs)
                    z = np.array([spd*math.sin(c), spd*math.cos(c)])
                    H = np.zeros((2, 9))
                    H[0, 3] = H[1, 4] = 1
                    ok, nis = ekf.update(z, H, cfg["gps_vel_sigma"]**2 * np.eye(2), cfg["gate_2dof"])
                    events.append(f"vel{'' if ok else '_rej'}:{nis:.1f}")

            cal = imu.calibration_status or (0, 0, 0, 0)
            sig = np.sqrt(np.diag(ekf.P)[0:3])
            log.writerow([f"{t:.4f}", f"{dt:.4f}", *np.round(ekf.x, 4), *np.round(sig, 3),
                          *np.round(a, 4), *np.round(g, 4), *np.round(q, 5), *cal,
                          "|".join(events)])

            if t - t_print > 0.5:
                t_print = t
                p, v, b = ekf.x[0:3], ekf.x[3:6], ekf.x[6:9]
                print(f"p {p[0]:7.2f} {p[1]:7.2f} {p[2]:6.2f} | "
                      f"v {v[0]:5.2f} {v[1]:5.2f} {v[2]:5.2f} | "
                      f"b {b[0]:+.3f} {b[1]:+.3f} {b[2]:+.3f} | "
                      f"{1/dt:3.0f} Hz cal {cal} hdop {hdop:.1f} sats {sats}")

            rest = period - (time.monotonic() - t_loop)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        pass
    finally:
        if gps:
            gps.stop.set()
        f.close()


if __name__ == "__main__":
    main()
