import time
import vicon_tracker
import numpy as np
import zenoh
import json
import queue
import random
import os


# ── Kalman filters (used by RelativePosePublisher for velocity estimation) ───

def _gate_innovation(innovation, S, gate_sigma):
    """Clip innovation to +/- gate_sigma*sqrt(S) so a single glitched measurement
    can't inject a step change into the filter state. gate_sigma <= 0 disables gating."""
    if gate_sigma <= 0.0:
        return innovation
    bound = gate_sigma * np.sqrt(S)
    return max(-bound, min(bound, innovation))


class KalmanCA1D:
    """1-D Constant-Acceleration Kalman filter. State: [pos, vel, acc]. Meas: pos."""
    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.eye(3) * 10.0
        self.initialized = False

    def update(self, z, dt, q, r, gate_sigma=0.0):
        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return 0.0
        dt2, dt3, dt4, dt5 = dt**2, dt**3, dt**4, dt**5
        F = np.array([[1, dt, 0.5*dt2],
                      [0,  1,      dt],
                      [0,  0,       1]])
        Q = np.array([[dt5/20, dt4/8, dt3/6],
                      [dt4/8,  dt3/3, dt2/2],
                      [dt3/6,  dt2/2,    dt]]) * q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        S = self.P[0, 0] + r
        innovation = _gate_innovation(z - self.x[0], S, gate_sigma)
        K = self.P[:, 0] / S
        self.x += K * innovation
        self.P -= np.outer(K, self.P[0, :])
        return self.x[1]


class KalmanCV1D:
    """1-D Constant-Velocity Kalman filter. State: [omega, alpha]. Meas: omega."""
    def __init__(self):
        self.x = np.zeros(2)
        self.P = np.eye(2) * 10.0
        self.initialized = False

    def update(self, z, dt, q, r, gate_sigma=0.0):
        if not self.initialized:
            self.x[0] = z
            self.initialized = True
            return z
        dt2, dt3 = dt**2, dt**3
        F = np.array([[1, dt], [0, 1]])
        Q = np.array([[dt3/3, dt2/2],
                      [dt2/2,    dt]]) * q
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        S = self.P[0, 0] + r
        innovation = _gate_innovation(z - self.x[0], S, gate_sigma)
        K = self.P[:, 0] / S
        self.x += K * innovation
        self.P -= np.outer(K, self.P[0, :])
        return self.x[0]


# ── Config parser ─────────────────────────────────────────────────────────────

def _trim_value(s):
    """Strip inline # comment and trailing whitespace."""
    idx = s.find('#')
    s = s[:idx] if idx != -1 else s
    return s.strip()


def load_section(config_file, section):
    """Parse a named [section] from an INI-style config file.
    Returns a dict of {key: value_string}."""
    result = {}
    in_section = False
    try:
        with open(config_file, 'r') as f:
            for raw in f:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                if line.startswith('['):
                    end = line.find(']')
                    sec = line[1:end].strip() if end != -1 else ''
                    in_section = (sec == section)
                    continue
                if not in_section:
                    continue
                sep = -1
                for i, ch in enumerate(line):
                    if ch in ':=':
                        sep = i
                        break
                if sep == -1:
                    continue
                key = line[:sep].strip()
                val = _trim_value(line[sep+1:])
                result[key] = val
    except FileNotFoundError:
        print(f"Config file not found: {config_file}")
    return result


# ── ObjectTracker ─────────────────────────────────────────────────────────────

class ObjectTracker:
    """Tracks a single VRPN object and publishes pose + fast pose to Zenoh."""

    R_sv = np.array([[0.0, -1.0, 0.0],
                     [1.0,  0.0, 0.0],
                     [0.0,  0.0, 1.0]])
    R_off = np.array([[-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0],
                      [0.0,  0.0, 1.0]])

    # NED-frame conversion: R_nv (vicon/ENU -> NED), R_im (marker -> IMU mounting).
    # Both are self-inverse/symmetric, so the same matrix works in either direction.
    R_nv = np.array([[-1.0, 0.0,  0.0],
                     [ 0.0, 1.0,  0.0],
                     [ 0.0, 0.0, -1.0]])
    R_im = np.array([[0.0, 1.0,  0.0],
                     [1.0, 0.0,  0.0],
                     [0.0, 0.0, -1.0]])

    def __init__(self, section_name):
        self.section = section_name
        self.on = False
        self.object_name = ''
        self.port = 3883
        self.zenoh_key = 'fdcl/pose_sync'
        self.latency = 0.0
        self.frequency = 5.0
        self.dt_desired = 0.2
        self.std_x = 0.0
        self.std_R = 0.0
        self.noise_x_enabled = False
        self.noise_R_enabled = False
        self.position_only = False
        self.fast_key = 'fdcl/pose_sync_fast'
        self.fast_enable = False
        self.fast_frequency = 200.0
        self.fast_dt = 1.0 / 200.0
        # When True, the fast_key publish keeps the legacy (pre-NED) R_sv/R_off transform
        # instead of the NED transform. x_clean_latest is always NED regardless of this flag.
        self.fast_legacy_frame = False

        # Clean (pre-noise) pose — shared with RelativePosePublisher
        self.x_clean_latest = np.zeros(3)
        self.R_clean_latest = np.eye(3)
        self.timestamp_clean_latest = 0
        self.has_data = False

        self._pose_buffer = queue.Queue()
        self._last_collect_mono = time.monotonic_ns()
        self._fast_last_mono = time.monotonic_ns()
        self._vicon = None
        self._session = None
        self._publisher = None
        self._fast_publisher = None
        self._rng = random.Random()

    def load_config(self, config_file, section=None):
        sec = section or self.section
        cfg = load_section(config_file, sec)

        def _bool(k, default=False):
            return cfg.get(k, str(default)).lower() in ('true', '1')
        def _float(k, default=0.0):
            try: return float(cfg.get(k, str(default)))
            except ValueError: return default
        def _int(k, default=0):
            try: return int(cfg.get(k, str(default)))
            except ValueError: return default

        self.object_name    = cfg.get('Object', '')
        self.port           = _int('Port', 3883)
        self.on             = _bool('on', False)
        self.zenoh_key      = cfg.get('zenoh_key', self.zenoh_key)
        self.latency        = _float('latency', 0.0)
        self.frequency      = _float('frequency', 5.0)
        self.dt_desired     = 1.0 / self.frequency if self.frequency > 0 else 0.2
        self.std_x          = _float('std_x', 0.0)
        self.std_R          = _float('std_R', 0.0)
        self.noise_x_enabled = _bool('noise_x_enabled', False)
        self.noise_R_enabled = _bool('noise_R_enabled', False)
        self.position_only  = _bool('position_only', False)
        self.fast_key       = cfg.get('fast_key', self.fast_key)
        self.fast_enable    = _bool('fast_enable', False)
        self.fast_frequency = _float('fast_frequency', 200.0)
        self.fast_dt        = 1.0 / self.fast_frequency if self.fast_frequency > 0 else 0.005
        self.fast_legacy_frame = _bool('fast_legacy_frame', False)

        print(f"[{sec}] Config: object={self.object_name}:{self.port} "
              f"zenoh_key={self.zenoh_key} on={self.on} "
              f"frequency={self.frequency}Hz latency={self.latency}s "
              f"fast={'on:'+self.fast_key if self.fast_enable else 'off'}"
              f"{' [fast:legacy_frame]' if self.fast_enable and self.fast_legacy_frame else ''}")

    def open(self):
        vrpn_name = f"{self.object_name}:{self.port}"
        try:
            self._vicon = vicon_tracker.vicon()
            self._vicon.open(vrpn_name)
            conf = zenoh.Config()
            self._session = zenoh.open(conf)
            self._publisher = self._session.declare_publisher(self.zenoh_key)
            print(f"[{self.section}] Tracker opened for {vrpn_name}, publisher on {self.zenoh_key}")
            if self.fast_enable:
                self._fast_publisher = self._session.declare_publisher(self.fast_key)
                print(f"[{self.section}] Fast publisher on {self.fast_key} ({int(self.fast_frequency)} Hz)")
        except Exception as e:
            print(f"[{self.section}] Failed to open: {e}")
            self.on = False

    def _to_pose_ned(self, x_v, R_vm):
        """NED/IMU-frame pose: x_n = R_nv @ x_v, R_ni = R_nv @ R_vm @ R_im."""
        x = self.R_nv @ x_v
        R = self.R_nv @ R_vm @ self.R_im
        return x, R

    def _to_pose_legacy(self, x_v, R_vm):
        x = self.R_sv @ x_v
        x[0] = -x[0]; x[1] = -x[1]
        R = self.R_off @ self.R_sv @ R_vm
        return x, R

    def _apply_noise(self, x, R):
        noise_x = np.zeros(3)
        noise_ang = np.zeros(3)
        if self.noise_x_enabled and self.std_x > 0.0:
            noise_x = np.array([self.std_x * (self._rng.random() * 2 - 1) for _ in range(3)])
            x = x + noise_x
        if self.noise_R_enabled and self.std_R > 0.0 and not self.position_only:
            noise_ang = np.array([self.std_R * (self._rng.random() * 2 - 1) for _ in range(3)])
            cx, sx = np.cos(noise_ang[0]), np.sin(noise_ang[0])
            cy, sy = np.cos(noise_ang[1]), np.sin(noise_ang[1])
            cz, sz = np.cos(noise_ang[2]), np.sin(noise_ang[2])
            Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            R = R @ Rz @ Ry @ Rx
        return x, R, noise_x, noise_ang

    def loop(self):
        if not self.on:
            return
        now_mono = time.monotonic_ns()
        ts_ns = time.time_ns()

        # Fast path
        if self.fast_enable and self._fast_publisher:
            if (now_mono - self._fast_last_mono) / 1e9 >= self.fast_dt:
                x_v, R_vm = self._vicon.loop()
                x_v, R_vm = np.array(x_v), np.array(R_vm)
                # NED pose always computed; feeds x_clean_latest (used by RelativePosePublisher)
                # regardless of which frame is actually published on fast_key below.
                x_ned, R_ned = self._to_pose_ned(x_v, R_vm)
                self.x_clean_latest = x_ned.copy()
                self.R_clean_latest = R_ned.copy()
                self.timestamp_clean_latest = ts_ns
                self.has_data = True

                if self.fast_legacy_frame:
                    xf, Rf = self._to_pose_legacy(x_v, R_vm)
                else:
                    xf, Rf = x_ned, R_ned
                pose_data = [[float(Rf[i, j]) for j in range(3)] + [float(xf[i])] for i in range(3)]
                payload = {'image_taken_time': ts_ns, 'pose': pose_data,
                           'noise_x': [0.0]*3, 'noise_angles': [0.0]*3}
                try:
                    self._fast_publisher.put(json.dumps(payload))
                except Exception as e:
                    print(f"[{self.section}] Fast publish error: {e}")
                self._fast_last_mono = now_mono

        # Main path
        if (now_mono - self._last_collect_mono) / 1e9 >= self.dt_desired:
            x_v, R_vm = self._vicon.loop()
            x_v, R_vm = np.array(x_v), np.array(R_vm)
            # NED/IMU-frame pose (always applied on the main path for both Base and Rover)
            x_raw, R_raw = self._to_pose_ned(x_v, R_vm)

            if not self.fast_enable:
                self.x_clean_latest = x_raw.copy()
                self.R_clean_latest = R_raw.copy()
                self.timestamp_clean_latest = ts_ns
                self.has_data = True

            R_use = -np.eye(3) if self.position_only else R_raw.copy()
            x_noisy, R_noisy, noise_x, noise_ang = self._apply_noise(x_raw.copy(), R_use)

            self._pose_buffer.put({
                'x_pose': x_noisy, 'R_pose': R_noisy,
                'timestamp': ts_ns, 'collect_mono': now_mono,
                'noise_x': noise_x, 'noise_angles': noise_ang
            })
            self._last_collect_mono = now_mono

        # Drain latency buffer
        while not self._pose_buffer.empty():
            front = self._pose_buffer.queue[0]
            elapsed = (time.monotonic_ns() - front['collect_mono']) / 1e9
            if elapsed < self.latency:
                break
            data = self._pose_buffer.get()
            pose_data = [[float(data['R_pose'][i, j]) for j in range(3)] + [float(data['x_pose'][i])]
                         for i in range(3)]
            payload = {
                'image_taken_time': data['timestamp'],
                'pose': pose_data,
                'noise_x': data['noise_x'].tolist(),
                'noise_angles': data['noise_angles'].tolist()
            }
            try:
                self._publisher.put(json.dumps(payload))
            except Exception as e:
                print(f"[{self.section}] Publish error: {e}")

    def close(self):
        self.on = False
        if self._vicon:
            self._vicon.close()
        if self._fast_publisher:
            del self._fast_publisher
        if self._publisher:
            del self._publisher
        if self._session:
            self._session.close()
        while not self._pose_buffer.empty():
            self._pose_buffer.get()
        print(f"[{self.section}] Closed")


# ── RelativePosePublisher ─────────────────────────────────────────────────────

class RelativePosePublisher:
    """Computes and publishes relative pose from two ObjectTrackers.
    GT state (Kalman-filtered velocity/omega) is published only here."""

    def __init__(self, base: ObjectTracker, rover: ObjectTracker):
        self._base  = base
        self._rover = rover
        self.on = False
        self.meas_type = 'rover2base'   # or 'base2rover'
        self.zenoh_key = 'fdcl/pose_sync'
        self.latency = 0.0
        self.frequency = 5.0
        self.dt_desired = 0.2
        self.std_x = 0.0
        self.std_R = 0.0
        self.noise_x_enabled = False
        self.noise_R_enabled = False
        self.position_only = False
        self.fast_key = 'fdcl/rel_pose_sync_fast'
        self.fast_enable = False
        self.fast_frequency = 200.0
        self.fast_dt = 1.0 / 200.0
        self.gt_state_key = 'fdcl/rel_gt_state'
        self.gt_state_enable = False
        # Published at its own independent rate (gt_freq), decoupled from `frequency` above
        self.gt_freq = 5.0
        self.gt_dt_desired = 0.2
        self.gt_transform_enable = False
        self.T_gt = np.eye(3)
        self.kf_q_pos = 0.01
        self.kf_r_pos = 1e-6
        self.kf_q_omega = 0.5
        self.kf_r_omega = 0.04
        # Spike rejection: clip innovation to this many sigma. 0 disables gating.
        self.kf_gate_sigma_pos = 5.0
        self.kf_gate_sigma_omega = 5.0
        # EMA post-filter time constants (seconds) — rate-invariant, unlike a fixed
        # per-sample alpha. Effective alpha = 1 - exp(-dt/tau).
        self.vel_smooth_tau_sec = 0.5
        self.omega_smooth_tau_sec = 0.3

        self._kf_pos   = [KalmanCA1D() for _ in range(3)]
        self._kf_omega = [KalmanCV1D() for _ in range(3)]
        self._v_smooth = np.zeros(3)
        self._w_smooth = np.zeros(3)
        self._smooth_init = False
        self._R_gt_prev = np.eye(3)
        self._gt_prev_ts = 0.0
        self._gt_has_prev = False

        self._pose_buffer = queue.Queue()
        self._last_collect_mono = time.monotonic_ns()
        self._fast_last_mono = time.monotonic_ns()
        self._gt_last_mono = time.monotonic_ns()
        self._session = None
        self._publisher = None
        self._fast_publisher = None
        self._gt_pub = None
        self._rng = random.Random()

    def load_config(self, config_file):
        cfg = load_section(config_file, 'Relative')

        def _bool(k, default=False):
            return cfg.get(k, str(default)).lower() in ('true', '1')
        def _float(k, default=0.0):
            try: return float(cfg.get(k, str(default)))
            except ValueError: return default

        self.on                 = _bool('on', False)
        # left = rover2base (default), right = base2rover
        raw_meas_type           = cfg.get('Measurement_type', 'rover2base').strip()
        self.meas_type          = 'base2rover' if raw_meas_type in ('base2rover', 'right') else 'rover2base'
        self.zenoh_key          = cfg.get('zenoh_key', self.zenoh_key)
        self.latency            = _float('latency', 0.0)
        self.frequency          = _float('frequency', 5.0)
        self.dt_desired         = 1.0 / self.frequency if self.frequency > 0 else 0.2
        self.std_x              = _float('std_x', 0.0)
        self.std_R              = _float('std_R', 0.0)
        self.noise_x_enabled    = _bool('noise_x_enabled', False)
        self.noise_R_enabled    = _bool('noise_R_enabled', False)
        self.position_only      = _bool('position_only', False)
        self.fast_key           = cfg.get('fast_key', self.fast_key)
        self.fast_enable        = _bool('fast_enable', False)
        self.fast_frequency     = _float('fast_frequency', 200.0)
        self.fast_dt            = 1.0 / self.fast_frequency if self.fast_frequency > 0 else 0.005
        self.gt_state_key       = cfg.get('gt_state_key', self.gt_state_key)
        self.gt_state_enable    = _bool('gt_state_enable', False)
        self.gt_freq            = _float('gt_freq', 5.0)
        self.gt_dt_desired      = 1.0 / self.gt_freq if self.gt_freq > 0 else 0.2
        self.gt_transform_enable = _bool('gt_transform_enable', False)
        self.kf_q_pos           = _float('kf_q_pos', 0.01)
        self.kf_r_pos           = _float('kf_r_pos', 1e-6)
        self.kf_q_omega         = _float('kf_q_omega', 0.5)
        self.kf_r_omega         = _float('kf_r_omega', 0.04)
        self.kf_gate_sigma_pos   = _float('kf_gate_sigma_pos', 5.0)
        self.kf_gate_sigma_omega = _float('kf_gate_sigma_omega', 5.0)
        self.vel_smooth_tau_sec   = _float('vel_smooth_tau_sec', 0.5)
        self.omega_smooth_tau_sec = _float('omega_smooth_tau_sec', 0.3)

        mat_str = cfg.get('gt_transform_matrix', '')
        if mat_str:
            try:
                vals = [float(v.strip()) for v in mat_str.split(',') if v.strip()]
                if len(vals) == 9:
                    self.T_gt = np.array(vals).reshape(3, 3)
                else:
                    print(f"[Relative] gt_transform_matrix needs 9 values, got {len(vals)}")
            except ValueError:
                print("[Relative] gt_transform_matrix parse error")

        print(f"[Relative] Config: type={self.meas_type} zenoh_key={self.zenoh_key} "
              f"on={self.on} frequency={self.frequency}Hz latency={self.latency}s "
              f"fast={'on:'+self.fast_key if self.fast_enable else 'off'} "
              f"gt={'on:'+self.gt_state_key if self.gt_state_enable else 'off'}")

    def open(self):
        try:
            conf = zenoh.Config()
            self._session = zenoh.open(conf)
            self._publisher = self._session.declare_publisher(self.zenoh_key)
            print(f"[Relative] publisher on {self.zenoh_key}")
            if self.fast_enable:
                self._fast_publisher = self._session.declare_publisher(self.fast_key)
                print(f"[Relative] fast publisher on {self.fast_key} ({int(self.fast_frequency)} Hz)")
            if self.gt_state_enable:
                self._gt_pub = self._session.declare_publisher(self.gt_state_key)
                print(f"[Relative] GT state publisher on {self.gt_state_key}")
        except Exception as e:
            print(f"[Relative] Failed to open: {e}")
            self.on = False

    def _compute_relative(self, x_b, R_b, x_r, R_r):
        if self.meas_type == 'rover2base':
            x_rel = R_b.T @ (x_r - x_b)
            R_rel = R_b.T @ R_r
        else:  # base2rover
            x_rel = R_r.T @ (x_b - x_r)
            R_rel = R_r.T @ R_b
        return x_rel, R_rel

    def _publish_gt_state(self, x_rel, R_rel, ts_sec):
        if not self._gt_pub:
            return
        if not self._gt_has_prev:
            for ax in range(3):
                self._kf_pos[ax].update(x_rel[ax], self.dt_desired, self.kf_q_pos, self.kf_r_pos,
                                         self.kf_gate_sigma_pos)
            self._R_gt_prev = R_rel.copy()
            self._gt_prev_ts = ts_sec
            self._gt_has_prev = True
            return

        dt = ts_sec - self._gt_prev_ts
        if dt <= 1e-6:
            return

        v_filt = np.array([self._kf_pos[ax].update(x_rel[ax], dt, self.kf_q_pos, self.kf_r_pos,
                                                     self.kf_gate_sigma_pos)
                           for ax in range(3)])

        Sk = R_rel.T @ ((R_rel - self._R_gt_prev) / dt)
        omega_raw = np.array([(Sk[2,1] - Sk[1,2]) / 2.0,
                               (Sk[0,2] - Sk[2,0]) / 2.0,
                               (Sk[1,0] - Sk[0,1]) / 2.0])
        omega_filt = np.array([self._kf_omega[ax].update(omega_raw[ax], dt, self.kf_q_omega, self.kf_r_omega,
                                                           self.kf_gate_sigma_omega)
                               for ax in range(3)])

        # EMA post-filter: alpha derived from a fixed time constant + actual dt, so
        # smoothing strength stays consistent regardless of gt_freq.
        if not self._smooth_init:
            self._v_smooth = v_filt.copy()
            self._w_smooth = omega_filt.copy()
            self._smooth_init = True
        else:
            vel_alpha   = 1.0 - np.exp(-dt / self.vel_smooth_tau_sec)
            omega_alpha = 1.0 - np.exp(-dt / self.omega_smooth_tau_sec)
            v_filt     = vel_alpha   * v_filt     + (1.0 - vel_alpha)   * self._v_smooth
            omega_filt = omega_alpha * omega_filt + (1.0 - omega_alpha) * self._w_smooth
            self._v_smooth = v_filt.copy()
            self._w_smooth = omega_filt.copy()

        self._R_gt_prev  = R_rel.copy()
        self._gt_prev_ts = ts_sec

        T = self.T_gt if self.gt_transform_enable else np.eye(3)
        pos_out   = T @ x_rel
        vel_out   = T @ v_filt
        R_out     = T @ R_rel @ T.T
        omega_out = T @ omega_filt

        payload = {
            'timestamp': ts_sec,
            'rel_pos':   pos_out.tolist(),
            'rel_vel':   vel_out.tolist(),
            'rel_R':     R_out.tolist(),
            'rel_omega': omega_out.tolist()
        }
        try:
            self._gt_pub.put(json.dumps(payload))
        except Exception as e:
            print(f"[Relative] GT state publish error: {e}")

    def _apply_noise(self, x, R):
        noise_x = np.zeros(3)
        noise_ang = np.zeros(3)
        if self.noise_x_enabled and self.std_x > 0.0:
            noise_x = np.array([self.std_x * (self._rng.random() * 2 - 1) for _ in range(3)])
            x = x + noise_x
        if self.noise_R_enabled and self.std_R > 0.0 and not self.position_only:
            noise_ang = np.array([self.std_R * (self._rng.random() * 2 - 1) for _ in range(3)])
            cx, sx = np.cos(noise_ang[0]), np.sin(noise_ang[0])
            cy, sy = np.cos(noise_ang[1]), np.sin(noise_ang[1])
            cz, sz = np.cos(noise_ang[2]), np.sin(noise_ang[2])
            Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
            Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
            Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
            R = R @ Rz @ Ry @ Rx
        return x, R, noise_x, noise_ang

    def loop(self):
        if not self.on:
            return
        if not self._base.has_data or not self._rover.has_data:
            return

        now_mono = time.monotonic_ns()
        ts_ns = time.time_ns()

        # Fast path: clean relative pose at fast rate
        if self.fast_enable and self._fast_publisher:
            if (now_mono - self._fast_last_mono) / 1e9 >= self.fast_dt:
                xf, Rf = self._compute_relative(
                    self._base.x_clean_latest,  self._base.R_clean_latest,
                    self._rover.x_clean_latest, self._rover.R_clean_latest)
                pose_data = [[float(Rf[i, j]) for j in range(3)] + [float(xf[i])] for i in range(3)]
                payload = {'image_taken_time': ts_ns, 'pose': pose_data,
                           'noise_x': [0.0]*3, 'noise_angles': [0.0]*3}
                try:
                    self._fast_publisher.put(json.dumps(payload))
                except Exception as e:
                    print(f"[Relative] Fast publish error: {e}")
                self._fast_last_mono = now_mono

        # GT state: published at its own independent rate (gt_freq), decoupled from
        # the noisy zenoh_key topic's rate. Always uses clean, noise-free relative pose.
        if self.gt_state_enable and self._gt_pub:
            if (now_mono - self._gt_last_mono) / 1e9 >= self.gt_dt_desired:
                x_gt_clean, R_gt_clean = self._compute_relative(
                    self._base.x_clean_latest,  self._base.R_clean_latest,
                    self._rover.x_clean_latest, self._rover.R_clean_latest)
                self._publish_gt_state(x_gt_clean, R_gt_clean, ts_ns / 1e9)
                self._gt_last_mono = now_mono

        # Main path
        if (now_mono - self._last_collect_mono) / 1e9 >= self.dt_desired:
            x_rel_clean, R_rel_clean = self._compute_relative(
                self._base.x_clean_latest,  self._base.R_clean_latest,
                self._rover.x_clean_latest, self._rover.R_clean_latest)

            R_use = -np.eye(3) if self.position_only else R_rel_clean.copy()
            x_noisy, R_noisy, noise_x, noise_ang = self._apply_noise(x_rel_clean.copy(), R_use)

            self._pose_buffer.put({
                'x_pose': x_noisy, 'R_pose': R_noisy,
                'timestamp': ts_ns, 'collect_mono': now_mono,
                'noise_x': noise_x, 'noise_angles': noise_ang
            })
            self._last_collect_mono = now_mono

        # Drain latency buffer
        while not self._pose_buffer.empty():
            front = self._pose_buffer.queue[0]
            elapsed = (time.monotonic_ns() - front['collect_mono']) / 1e9
            if elapsed < self.latency:
                break
            data = self._pose_buffer.get()
            pose_data = [[float(data['R_pose'][i, j]) for j in range(3)] + [float(data['x_pose'][i])]
                         for i in range(3)]
            payload = {
                'image_taken_time': data['timestamp'],
                'pose': pose_data,
                'noise_x': data['noise_x'].tolist(),
                'noise_angles': data['noise_angles'].tolist()
            }
            try:
                self._publisher.put(json.dumps(payload))
            except Exception as e:
                print(f"[Relative] Publish error: {e}")

    def close(self):
        self.on = False
        if self._gt_pub:
            del self._gt_pub
        if self._fast_publisher:
            del self._fast_publisher
        if self._publisher:
            del self._publisher
        if self._session:
            self._session.close()
        while not self._pose_buffer.empty():
            self._pose_buffer.get()
        print("[Relative] Closed")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cfg = os.path.join(script_dir, '..', 'config.cfg')

    base  = ObjectTracker('Base')
    base.load_config(cfg, 'Base')

    rover = ObjectTracker('Rover')
    rover.load_config(cfg, 'Rover')

    rel = RelativePosePublisher(base, rover)
    rel.load_config(cfg)

    if base.on:  base.open()
    if rover.on: rover.open()
    if rel.on:   rel.open()

    print("Starting vicon2zenoh (base + rover + relative)... Press Ctrl+C to stop.")
    try:
        while True:
            if base.on:  base.loop()
            if rover.on: rover.loop()
            if rel.on:   rel.loop()
            # Tighter poll than 1ms so 200Hz deadlines (5ms period) aren't missed by a
            # large margin; note Python's interpreter/GIL overhead means this will still
            # track 200Hz less tightly than the C++ build.
            time.sleep(0.0001)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if base.on:  base.close()
        if rover.on: rover.close()
        if rel.on:   rel.close()


if __name__ == '__main__':
    main()
