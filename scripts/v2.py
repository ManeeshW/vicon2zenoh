import math
import time
import vicon_tracker
import numpy as np
import zenoh
import json
import queue
import random

# Fixed frame transforms (Vicon -> body frame)
_R_SV = np.array([[0.0, -1.0, 0.0],
                  [1.0,  0.0, 0.0],
                  [0.0,  0.0, 1.0]])
_R_OFF = np.array([[-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0,  0.0, 1.0]])
_VICON_TO_BODY = _R_OFF @ _R_SV  # [[0,1,0],[-1,0,0],[0,0,1]]


# ---------------------------------------------------------------------------
# Kalman filters
# ---------------------------------------------------------------------------

class KalmanCA1D:
    """
    1-D Constant-Acceleration Kalman filter.
    State: [pos, vel, acc].  Measurement: position.
    Returns filtered velocity.
    """
    def __init__(self):
        self.x = np.zeros(3)                    # [pos, vel, acc]
        self.P = np.eye(3) * 10.0
        self.initialized = False

    def update(self, z, dt, q, r):
        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return 0.0

        dt2, dt3, dt4, dt5 = dt**2, dt**3, dt**4, dt**5
        F = np.array([[1, dt, 0.5*dt2],
                      [0,  1,      dt],
                      [0,  0,       1]])
        Q = q * np.array([[dt5/20, dt4/8, dt3/6],
                           [dt4/8,  dt3/3, dt2/2],
                           [dt3/6,  dt2/2,    dt]])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Update  H = [1, 0, 0]
        S = self.P[0, 0] + r
        K = self.P[:, 0] / S
        self.x += K * (z - self.x[0])
        self.P -= np.outer(K, self.P[0, :])

        return self.x[1]   # filtered velocity


class KalmanCV1D:
    """
    1-D Constant-Velocity Kalman filter.
    State: [omega, alpha].  Measurement: raw finite-difference omega.
    Returns smoothed omega.
    """
    def __init__(self):
        self.x = np.zeros(2)                    # [omega, angular_acc]
        self.P = np.eye(2) * 10.0
        self.initialized = False

    def update(self, z, dt, q, r):
        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return z

        dt2, dt3 = dt**2, dt**3
        F = np.array([[1, dt],
                      [0,  1]])
        Q = q * np.array([[dt3/3, dt2/2],
                           [dt2/2,    dt]])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        # Update  H = [1, 0]
        S = self.P[0, 0] + r
        K = self.P[:, 0] / S
        self.x += K * (z - self.x[0])
        self.P -= np.outer(K, self.P[0, :])

        return self.x[0]   # smoothed omega


# ---------------------------------------------------------------------------

def _apply_noise(x, R, std_x, std_R, noise_x_en, noise_R_en, rng):
    noise_x = np.zeros(3)
    noise_a = np.zeros(3)
    if noise_x_en and std_x > 0:
        noise_x = np.array([std_x * (rng.random() * 2 - 1) for _ in range(3)])
        x = x + noise_x
    if noise_R_en and std_R > 0:
        noise_a = np.array([std_R * (rng.random() * 2 - 1) for _ in range(3)])
        cx, sx = np.cos(noise_a[0]), np.sin(noise_a[0])
        cy, sy = np.cos(noise_a[1]), np.sin(noise_a[1])
        cz, sz = np.cos(noise_a[2]), np.sin(noise_a[2])
        Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        R = R @ Rz @ Ry @ Rx
    return x, R, noise_x, noise_a


class ObjectTracker:
    """Tracks one Vicon object and publishes its pose to a dedicated Zenoh key."""

    def __init__(self, name, key, session, cfg):
        self.name = name
        self.key = key
        self.latency = cfg["latency"]
        self.dt_desired = cfg["dt_desired"]
        self.std_x = cfg["std_x"]
        self.std_R = cfg["std_R"]
        self.noise_x_enabled = cfg["noise_x_enabled"]
        self.noise_R_enabled = cfg["noise_R_enabled"]
        self.timestamp_offset = cfg["timestamp_offset"]

        self.x_pose = np.zeros(3)
        self.R_pose = np.eye(3)
        self.timestamp = 0
        self.has_data = False
        self.updated = False

        self._last_collect_ns = time.monotonic_ns()
        self._buffer = queue.Queue()
        self._rng = random.Random()

        self._vicon = vicon_tracker.vicon()
        self._vicon.open(name)
        self._pub = session.declare_publisher(key)
        print(f"[ObjectTracker] '{name}' -> '{key}'")

    def loop(self):
        self.updated = False
        now_ns = time.monotonic_ns()
        if (now_ns - self._last_collect_ns) / 1e9 >= self.dt_desired:
            x_v, R_vm = self._vicon.loop()
            x = _VICON_TO_BODY @ np.array(x_v)
            R = _VICON_TO_BODY @ np.array(R_vm)
            ts = time.time_ns() + int(self.timestamp_offset * 1e9)
            x, R, noise_x, noise_a = _apply_noise(
                x, R, self.std_x, self.std_R,
                self.noise_x_enabled, self.noise_R_enabled, self._rng
            )
            self._buffer.put({"x": x, "R": R, "ts": ts, "t0": now_ns,
                               "noise_x": noise_x, "noise_a": noise_a})
            self.x_pose = x
            self.R_pose = R
            self.timestamp = ts
            self.has_data = True
            self.updated = True
            self._last_collect_ns = now_ns

        # Drain buffer: publish entries that have waited >= latency
        while not self._buffer.empty():
            d = self._buffer.queue[0]
            if (time.monotonic_ns() - d["t0"]) / 1e9 >= self.latency:
                pose_mat = [[float(d["R"][i, j]) for j in range(3)] + [float(d["x"][i])]
                            for i in range(3)]
                payload = json.dumps({
                    "image_taken_time": d["ts"],
                    "pose": pose_mat,
                    "noise_x": d["noise_x"].tolist(),
                    "noise_angles": d["noise_a"].tolist()
                })
                try:
                    self._pub.put(payload)
                    R = d["R"]
                    roll  = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
                    yaw   = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    print(f"[{self.key}] x={d['x']}  rpy_deg=[{roll:.2f}, {pitch:.2f}, {yaw:.2f}]")
                except Exception as e:
                    print(f"[{self.key}] publish error: {e}")
                self._buffer.get()
            else:
                break

    def close(self):
        self._vicon.close()
        del self._pub
        while not self._buffer.empty():
            self._buffer.get()
        print(f"[ObjectTracker] Closed '{self.name}'")


class Vicon2Zenoh:
    """
    Multi-object Vicon bridge.

    Publishes each object to  fdcl/object<N>  and the relative pose
    (object[pose_sync_to] expressed in frame of object[pose_sync_from])
    to  fdcl/pose_sync.

    Also publishes Kalman-smoothed relative state (pos + vel + omega + R)
    to  fdcl/rel_gt_state  with an optional output-frame transform.
    """

    _DEFAULTS = dict(
        on=True,
        latency=0.0,
        frequency=200.0,
        std_x=0.0,
        std_R=0.0,
        noise_x_enabled=False,
        noise_R_enabled=False,
        timestamp_offset=0.0,
        object_names=["OriginsX@192.168.10.1", "OriginsY@192.168.10.1"],
        object_keys=["fdcl/object1", "fdcl/object2"],
        pose_sync_from=0,
        pose_sync_to=1,
        pose_sync_key="fdcl/pose_sync",
        rel_gt_state_key="fdcl/rel_gt_state",
        rel_gt_state_enable=True,
        rel_pose_transform_enable=True,
        kf_q_pos=1.0,
        kf_r_pos=1e-6,
        kf_q_omega=5.0,
        kf_r_omega=0.04,
    )

    _DEFAULT_T_REL = np.eye(3)

    def __init__(self, config_file="../config_py.cfg"):
        for k, v in self._DEFAULTS.items():
            setattr(self, k, v)
        self.T_rel = self._DEFAULT_T_REL.copy()
        self.dt_desired = 1.0 / self.frequency
        self._object_overrides = {}  # {int index -> dict of override values}

        self._trackers = []
        self._session = None
        self._sync_pub = None
        self._rel_gt_state_pub = None
        self._rel_buffer = queue.Queue()

        # Kalman filter instances (3 per axis for pos→vel, 3 for omega)
        self._kf_pos   = [KalmanCA1D() for _ in range(3)]
        self._kf_omega = [KalmanCV1D() for _ in range(3)]

        # Previous relative rotation for angular velocity finite difference
        self._R_rel_prev   = np.eye(3)
        self._rel_prev_ts  = 0.0
        self._rel_has_prev = False

        self._load_config(config_file)

    def _load_config(self, path):
        try:
            with open(path) as f:
                for raw in f:
                    line = raw.strip()
                    if not line or line.startswith("#"):
                        continue
                    key, _, val = line.partition(":")
                    key, val = key.strip(), val.strip()
                    if key == "objects":
                        self.object_names = [o.strip() for o in val.split(",") if o.strip()]
                    elif key == "object":
                        self.object_names = [val.strip('"')]
                    elif key == "object_keys":
                        self.object_keys = [k.strip() for k in val.split(",") if k.strip()]
                    elif key == "pose_sync_from":
                        self.pose_sync_from = int(val)
                    elif key == "pose_sync_to":
                        self.pose_sync_to = int(val)
                    elif key == "pose_sync_key":
                        self.pose_sync_key = val
                    elif key == "latency":
                        self.latency = float(val)
                    elif key == "frequency":
                        self.frequency = float(val)
                        self.dt_desired = 1.0 / self.frequency
                    elif key == "on":
                        self.on = val.lower() in ("true", "1")
                    elif key == "std_x":
                        self.std_x = float(val)
                    elif key == "std_R":
                        self.std_R = float(val)
                    elif key == "noise_x_enabled":
                        self.noise_x_enabled = val.lower() in ("true", "1")
                    elif key == "noise_R_enabled":
                        self.noise_R_enabled = val.lower() in ("true", "1")
                    elif key == "timestamp_offset":
                        self.timestamp_offset = float(val)
                    elif key == "rel_gt_state_key":
                        self.rel_gt_state_key = val
                    elif key == "rel_gt_state_enable":
                        self.rel_gt_state_enable = val.lower() in ("true", "1")
                    elif key == "rel_pose_transform_enable":
                        self.rel_pose_transform_enable = val.lower() in ("true", "1")
                    elif key == "rel_pose_transform":
                        vals = [float(x.strip()) for x in val.split(",")]
                        if len(vals) == 9:
                            self.T_rel = np.array(vals, dtype=float).reshape(3, 3)
                    elif key == "kf_q_pos":
                        self.kf_q_pos = float(val)
                    elif key == "kf_r_pos":
                        self.kf_r_pos = float(val)
                    elif key == "kf_q_omega":
                        self.kf_q_omega = float(val)
                    elif key == "kf_r_omega":
                        self.kf_r_omega = float(val)
                    elif key.startswith("object_") and len(key) > 7 and key[7].isdigit():
                        rest = key[7:]
                        ul = rest.find("_")
                        if ul != -1:
                            try:
                                idx = int(rest[:ul])
                                setting = rest[ul + 1:]
                                ov = self._object_overrides.setdefault(idx, {})
                                if setting == "frequency":
                                    ov["frequency"] = float(val)
                                elif setting == "latency":
                                    ov["latency"] = float(val)
                                elif setting == "std_x":
                                    ov["std_x"] = float(val)
                                elif setting == "std_R":
                                    ov["std_R"] = float(val)
                                elif setting == "noise_x_enabled":
                                    ov["noise_x_enabled"] = val.lower() in ("true", "1")
                                elif setting == "noise_R_enabled":
                                    ov["noise_R_enabled"] = val.lower() in ("true", "1")
                            except (ValueError, IndexError):
                                pass
            rgs = self.rel_gt_state_key if self.rel_gt_state_enable else "disabled"
            T_str = f"on {self.T_rel.tolist()}" if self.rel_pose_transform_enable else "off"
            print(
                f"[Vicon2Zenoh] Config: {len(self.object_names)} objects, "
                f"f={self.frequency}Hz, latency={self.latency}s, "
                f"pose_sync [{self.pose_sync_from}]->[{self.pose_sync_to}] -> '{self.pose_sync_key}'\n"
                f"  rel_gt_state={rgs}  transform={T_str}\n"
                f"  KF: q_pos={self.kf_q_pos} r_pos={self.kf_r_pos} "
                f"q_omega={self.kf_q_omega} r_omega={self.kf_r_omega}"
            )
        except FileNotFoundError:
            print(f"[Vicon2Zenoh] Config not found at '{path}', using defaults")
        except Exception as e:
            print(f"[Vicon2Zenoh] Config error: {e}")

    def _tracker_cfg(self, idx):
        cfg = dict(
            latency=self.latency,
            dt_desired=self.dt_desired,
            std_x=self.std_x,
            std_R=self.std_R,
            noise_x_enabled=self.noise_x_enabled,
            noise_R_enabled=self.noise_R_enabled,
            timestamp_offset=self.timestamp_offset,
        )
        ov = self._object_overrides.get(idx, {})
        if "frequency" in ov:
            cfg["dt_desired"] = 1.0 / ov["frequency"]
        for field in ("latency", "std_x", "std_R", "noise_x_enabled", "noise_R_enabled"):
            if field in ov:
                cfg[field] = ov[field]
        if ov:
            eff_freq = 1.0 / cfg["dt_desired"]
            print(f"[Vicon2Zenoh] object_{idx} overrides: f={eff_freq}Hz, latency={cfg['latency']}s")
        return cfg

    def open(self):
        try:
            self._session = zenoh.open(zenoh.Config())
            self._sync_pub = self._session.declare_publisher(self.pose_sync_key)
            if self.rel_gt_state_enable:
                self._rel_gt_state_pub = self._session.declare_publisher(self.rel_gt_state_key)
                print(f"[Vicon2Zenoh] rel_gt_state -> '{self.rel_gt_state_key}'")

            while len(self.object_keys) < len(self.object_names):
                self.object_keys.append(f"fdcl/object{len(self.object_keys) + 1}")

            for i, (name, key) in enumerate(zip(self.object_names, self.object_keys)):
                cfg = self._tracker_cfg(i)
                self._trackers.append(ObjectTracker(name, key, self._session, cfg))

            print(
                f"[Vicon2Zenoh] {len(self._trackers)} trackers opened, "
                f"relative pose on '{self.pose_sync_key}'"
            )
        except Exception as e:
            print(f"[Vicon2Zenoh] Open failed: {e}")
            self.on = False

    def _queue_and_drain_relative_pose(self):
        fi = self.pose_sync_from
        ti = self.pose_sync_to
        if fi >= len(self._trackers) or ti >= len(self._trackers):
            return
        from_t = self._trackers[fi]
        to_t = self._trackers[ti]
        if not (from_t.has_data and to_t.has_data):
            return

        if from_t.updated or to_t.updated:
            x_rel = from_t.R_pose.T @ (to_t.x_pose - from_t.x_pose)
            R_rel = from_t.R_pose.T @ to_t.R_pose
            ts = max(from_t.timestamp, to_t.timestamp)
            ts_sec = ts / 1e9

            # Queue for pose_sync (honours latency)
            self._rel_buffer.put({"x": x_rel, "R": R_rel, "ts": ts,
                                   "t0": time.monotonic_ns()})

            # --- rel_gt_state: zero-latency, Kalman-filtered velocity ---
            if self.rel_gt_state_enable and self._rel_gt_state_pub:
                if not self._rel_has_prev:
                    # Seed KF position states on first sample, skip publishing
                    for ax in range(3):
                        self._kf_pos[ax].update(x_rel[ax], self.dt_desired,
                                                 self.kf_q_pos, self.kf_r_pos)
                    self._R_rel_prev  = R_rel.copy()
                    self._rel_prev_ts = ts_sec
                    self._rel_has_prev = True
                else:
                    dt = ts_sec - self._rel_prev_ts
                    if dt > 1e-6:
                        # Kalman-filtered velocity (CA model on position)
                        v_filt = np.array([
                            self._kf_pos[ax].update(x_rel[ax], dt,
                                                     self.kf_q_pos, self.kf_r_pos)
                            for ax in range(3)
                        ])

                        # Finite-difference angular velocity, Kalman-smoothed (CV model)
                        Sk = R_rel.T @ ((R_rel - self._R_rel_prev) / dt)
                        omega_raw = np.array([
                            (Sk[2, 1] - Sk[1, 2]) / 2.0,
                            (Sk[0, 2] - Sk[2, 0]) / 2.0,
                            (Sk[1, 0] - Sk[0, 1]) / 2.0,
                        ])
                        omega_filt = np.array([
                            self._kf_omega[ax].update(omega_raw[ax], dt,
                                                       self.kf_q_omega, self.kf_r_omega)
                            for ax in range(3)
                        ])

                        self._R_rel_prev  = R_rel.copy()
                        self._rel_prev_ts = ts_sec

                        # Apply output-frame transform: T * v, T * R * T^T (T is symmetric).
                        # omega is a pseudovector and requires the det(T) factor under improper T.
                        T = self.T_rel if self.rel_pose_transform_enable else np.eye(3)
                        pos_out   = T @ x_rel
                        vel_out   = T @ v_filt
                        R_out     = T @ R_rel @ T.T
                        omega_out = np.linalg.det(T) * T @ omega_filt

                        R_mat = [[float(R_out[r, c]) for c in range(3)] for r in range(3)]
                        payload = json.dumps({
                            "timestamp": ts_sec,
                            "rel_pos":   pos_out.tolist(),
                            "rel_vel":   vel_out.tolist(),
                            "rel_omega": omega_out.tolist(),
                            "rel_R":     R_mat,
                        })
                        try:
                            self._rel_gt_state_pub.put(payload)
                        except Exception as e:
                            print(f"[{self.rel_gt_state_key}] publish error: {e}")

        # Drain buffer for pose_sync
        while not self._rel_buffer.empty():
            d = self._rel_buffer.queue[0]
            if (time.monotonic_ns() - d["t0"]) / 1e9 >= self.latency:
                pose_mat = [[float(d["R"][i, j]) for j in range(3)] + [float(d["x"][i])]
                            for i in range(3)]
                payload = json.dumps({
                    "image_taken_time": d["ts"],
                    "pose": pose_mat,
                    "noise_x": [0.0, 0.0, 0.0],
                    "noise_angles": [0.0, 0.0, 0.0],
                    "from_key": self.object_keys[fi],
                    "to_key": self.object_keys[ti]
                })
                try:
                    self._sync_pub.put(payload)
                    R = d["R"]
                    roll  = math.degrees(math.atan2(R[2, 1], R[2, 2]))
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
                    yaw   = math.degrees(math.atan2(R[1, 0], R[0, 0]))
                    print(
                        f"[{self.pose_sync_key}] "
                        f"'{self.object_keys[fi]}'->'{self.object_keys[ti]}' "
                        f"x_rel={d['x']}  rpy_deg=[{roll:.2f}, {pitch:.2f}, {yaw:.2f}]"
                    )
                except Exception as e:
                    print(f"[{self.pose_sync_key}] publish error: {e}")
                self._rel_buffer.get()
            else:
                break

    def loop(self):
        if not self.on:
            return
        for t in self._trackers:
            t.loop()
        self._queue_and_drain_relative_pose()

    def close(self):
        self.on = False
        for t in self._trackers:
            t.close()
        while not self._rel_buffer.empty():
            self._rel_buffer.get()
        if self._sync_pub:
            del self._sync_pub
        if self._rel_gt_state_pub:
            del self._rel_gt_state_pub
        self._rel_has_prev = False
        if self._session:
            self._session.close()
        print("[Vicon2Zenoh] Closed")


def main():
    v2z = Vicon2Zenoh()
    if not v2z.on:
        print("[Vicon2Zenoh] Disabled in config, exiting")
        return
    v2z.open()
    try:
        print("[Vicon2Zenoh] Running... Press Ctrl+C to stop.")
        while v2z.on:
            v2z.loop()
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("[Vicon2Zenoh] Interrupted")
    finally:
        v2z.close()


if __name__ == "__main__":
    main()
