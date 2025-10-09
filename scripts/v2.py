import time
import vicon_tracker
import numpy as np
import os
import zenoh
import json
import queue
import random

class Vicon2Pose:
    def __init__(self):
        self.on = True
        self.zenoh_key = "fdcl/pose_sync"
        self.latency = 0.5
        self.frequency = 5.0
        self.dt_desired = 0.2
        self.std_x = 0.0
        self.std_R = 0.0
        self.noise_x_enabled = False
        self.noise_R_enabled = False
        self.switch_vicon2pose = False
        self.timestamp_offset = 0.2
        self.R_sv = np.array([[0.0, -1.0, 0.0],
                              [1.0,  0.0, 0.0],
                              [0.0,  0.0, 1.0]])
        self.R_off = np.array([[-1.0, 0.0, 0.0],
                               [0.0, -1.0, 0.0],
                               [0.0, 0.0, 1.0]])
        self.x_v = np.zeros(3)
        self.R_vm = np.eye(3)
        self.x_pose_sync = np.zeros(3)
        self.R_pose_sync = np.eye(3)
        self.timestamp = 0
        self.last_collect_time = time.monotonic_ns()
        self.pose_buffer = queue.Queue()
        self.vicon = None
        self.session = None
        self.publisher = None
        self.subscriber = None
        self.rng = random.Random()
        self.load_config("../config_py.cfg")

    def load_config(self, config_file):
        self.object_name = "OriginsX@192.168.10.1"  # Default
        try:
            with open(config_file, 'r') as file:
                for line in file:
                    line = line.strip()
                    if not line:
                        continue
                    if line.startswith("object:"):
                        self.object_name = line.split("object:")[1].strip().strip('"')
                    elif line.startswith("zenoh_key:"):
                        self.zenoh_key = line.split("zenoh_key:")[1].strip().strip('"')
                    elif line.startswith("latency:"):
                        try:
                            self.latency = float(line.split("latency:")[1].strip())
                        except ValueError:
                            print(f"VICON2POSE: Invalid latency value, using default: 0.5")
                            self.latency = 0.5
                    elif line.startswith("frequency:"):
                        try:
                            self.frequency = float(line.split("frequency:")[1].strip())
                            self.dt_desired = 1.0 / self.frequency
                        except ValueError:
                            print(f"VICON2POSE: Invalid frequency value, using default: 5.0")
                            self.frequency = 5.0
                            self.dt_desired = 0.2
                    elif line.startswith("on:"):
                        value = line.split("on:")[1].strip().lower()
                        self.on = value in ("true", "1")
                    elif line.startswith("std_x:"):
                        try:
                            self.std_x = float(line.split("std_x:")[1].strip())
                        except ValueError:
                            print(f"VICON2POSE: Invalid std_x value, using default: 0.0")
                            self.std_x = 0.0
                    elif line.startswith("std_R:"):
                        try:
                            self.std_R = float(line.split("std_R:")[1].strip())
                        except ValueError:
                            print(f"VICON2POSE: Invalid std_R value, using default: 0.0")
                            self.std_R = 0.0
                    elif line.startswith("noise_x_enabled:"):
                        value = line.split("noise_x_enabled:")[1].strip().lower()
                        self.noise_x_enabled = value in ("true", "1")
                    elif line.startswith("noise_R_enabled:"):
                        value = line.split("noise_R_enabled:")[1].strip().lower()
                        self.noise_R_enabled = value in ("true", "1")
                    elif line.startswith("switch_vicon2pose:"):
                        value = line.split("switch_vicon2pose:")[1].strip().lower()
                        self.switch_vicon2pose = value in ("true", "1")
                    elif line.startswith("timestamp_offset:"):
                        try:
                            self.timestamp_offset = float(line.split("timestamp_offset:")[1].strip())
                        except ValueError:
                            print(f"VICON2POSE: Invalid timestamp_offset value, using default: 0.2")
                            self.timestamp_offset = 0.2
            print(f"VICON2POSE: Loaded config - object: {self.object_name}, zenoh_key: {self.zenoh_key}, "
                  f"latency: {self.latency}s, frequency: {self.frequency}Hz, dt_desired: {self.dt_desired}s, "
                  f"on: {self.on}, std_x: {self.std_x}, std_R: {self.std_R}, noise_x_enabled: {self.noise_x_enabled}, "
                  f"noise_R_enabled: {self.noise_R_enabled}, switch_vicon2pose: {self.switch_vicon2pose}, "
                  f"timestamp_offset: {self.timestamp_offset}s")
        except FileNotFoundError:
            print(f"VICON2POSE: Config file {config_file} not found, using defaults")
        except Exception as e:
            print(f"VICON2POSE: Error reading config file: {e}, using defaults")

    def open(self):
        try:
            conf = zenoh.Config()
            self.session = zenoh.open(conf)
            self.publisher = self.session.declare_publisher(self.zenoh_key)
            if self.switch_vicon2pose:
                self.subscriber = self.session.declare_subscriber("fdcl/pose_", self.on_receive)
                print(f"VICON2POSE: Opened Zenoh subscriber on fdcl/pose and publisher on {self.zenoh_key}")
            else:
                self.vicon = vicon_tracker.vicon()
                self.vicon.open(self.object_name)
                print(f"VICON2POSE: Opened Vicon tracker for {self.object_name} and Zenoh publisher on {self.zenoh_key}")
        except Exception as e:
            print(f"VICON2POSE: Failed to open - {e}")
            self.on = False

    def on_receive(self, sample):
        try:
            payload = sample.payload.to_string()
            received_data = json.loads(payload)
            pose_list = received_data["pose"]
            pose = np.array(pose_list)
            R_pose = pose[:, :3]
            x_pose = pose[:, 3]
            timestamp_ns = received_data["image_taken_time"] + int(self.timestamp_offset * 1_000_000_000)
            noise_x = np.zeros(3)
            noise_angles = np.zeros(3)

            # Add noise to position if enabled
            if self.noise_x_enabled and self.std_x > 0.0:
                noise_x = np.array([self.std_x * (self.rng.random() * 2 - 1) for _ in range(3)])
                x_pose += noise_x
                print(f"VICON2POSE: Applied position noise: {noise_x}")

            # Add noise to rotation if enabled
            if self.noise_R_enabled and self.std_R > 0.0:
                noise_angles = np.array([self.std_R * (self.rng.random() * 2 - 1) for _ in range(3)])
                cx, sx = np.cos(noise_angles[0]), np.sin(noise_angles[0])
                cy, sy = np.cos(noise_angles[1]), np.sin(noise_angles[1])
                cz, sz = np.cos(noise_angles[2]), np.sin(noise_angles[2])
                Rx = np.array([[1.0, 0.0, 0.0],
                               [0.0, cx, -sx],
                               [0.0, sx, cx]])
                Ry = np.array([[cy, 0.0, sy],
                               [0.0, 1.0, 0.0],
                               [-sy, 0.0, cy]])
                Rz = np.array([[cz, -sz, 0.0],
                               [sz, cz, 0.0],
                               [0.0, 0.0, 1.0]])
                R_pose = R_pose @ Rz @ Ry @ Rx
                print(f"VICON2POSE: Applied rotation noise angles (rad): {noise_angles}")

            collect_time_ns = time.monotonic_ns()
            self.pose_buffer.put({
                "x_pose": x_pose,
                "R_pose": R_pose,
                "timestamp": timestamp_ns,
                "collect_time": collect_time_ns,
                "noise_x": noise_x,
                "noise_angles": noise_angles
            })
            self.x_pose_sync = x_pose
            self.R_pose_sync = R_pose
            self.timestamp = timestamp_ns
            print(f"VICON2POSE: Received and queued - t: {timestamp_ns}, x_pose: {x_pose}, R_pose:\n{R_pose}")
        except Exception as e:
            print(f"VICON2POSE: Error in on_receive - {e}")

    def loop(self):
        if not self.on:
            return
        current_monotonic_ns = time.monotonic_ns()
        if not self.switch_vicon2pose:
            dt_ns = (current_monotonic_ns - self.last_collect_time)
            if dt_ns / 1e9 >= self.dt_desired:
                x_v, R_vm = self.vicon.loop()
                self.x_v = np.array(x_v)
                self.R_vm = np.array(R_vm)
                x_pose = self.R_off @ self.R_sv @ self.x_v
                R_pose = self.R_off @ self.R_sv @ self.R_vm
                timestamp_ns = time.time_ns() + int(self.timestamp_offset * 1_000_000_000)
                collect_time_ns = current_monotonic_ns
                noise_x = np.zeros(3)
                noise_angles = np.zeros(3)

                # Add noise to position if enabled
                if self.noise_x_enabled and self.std_x > 0.0:
                    noise_x = np.array([self.std_x * (self.rng.random() * 2 - 1) for _ in range(3)])
                    x_pose += noise_x
                    print(f"VICON2POSE: Applied position noise: {noise_x}")

                # Add noise to rotation if enabled
                if self.noise_R_enabled and self.std_R > 0.0:
                    noise_angles = np.array([self.std_R * (self.rng.random() * 2 - 1) for _ in range(3)])
                    cx, sx = np.cos(noise_angles[0]), np.sin(noise_angles[0])
                    cy, sy = np.cos(noise_angles[1]), np.sin(noise_angles[1])
                    cz, sz = np.cos(noise_angles[2]), np.sin(noise_angles[2])
                    Rx = np.array([[1.0, 0.0, 0.0],
                                   [0.0, cx, -sx],
                                   [0.0, sx, cx]])
                    Ry = np.array([[cy, 0.0, sy],
                                   [0.0, 1.0, 0.0],
                                   [-sy, 0.0, cy]])
                    Rz = np.array([[cz, -sz, 0.0],
                                   [sz, cz, 0.0],
                                   [0.0, 0.0, 1.0]])
                    R_pose = R_pose @ Rz @ Ry @ Rx
                    print(f"VICON2POSE: Applied rotation noise angles (rad): {noise_angles}")

                self.pose_buffer.put({
                    "x_pose": x_pose,
                    "R_pose": R_pose,
                    "timestamp": timestamp_ns,
                    "collect_time": collect_time_ns,
                    "noise_x": noise_x,
                    "noise_angles": noise_angles
                })
                self.x_pose_sync = x_pose
                self.R_pose_sync = R_pose
                self.timestamp = timestamp_ns
                self.last_collect_time = current_monotonic_ns

        # Publish data that has reached the latency delay
        while not self.pose_buffer.empty():
            data = self.pose_buffer.queue[0]
            time_since_collect = (time.monotonic_ns() - data["collect_time"]) / 1e9
            if time_since_collect >= self.latency:
                pose_data = [[float(data["R_pose"][i, j]) for j in range(3)] + [float(data["x_pose"][i])] for i in range(3)]
                payload = {
                    "image_taken_time": data["timestamp"],
                    "pose": pose_data,
                    "noise_x": data["noise_x"].tolist(),
                    "noise_angles": data["noise_angles"].tolist()
                }
                try:
                    self.publisher.put(json.dumps(payload))
                    print(f"VICON2POSE: Published - t: {data['timestamp']}, x_pose_sync: {data['x_pose']}, "
                          f"R_pose_sync:\n{data['R_pose']}, noise_x: {data['noise_x']}, "
                          f"noise_angles: {data['noise_angles']}")
                except Exception as e:
                    print(f"VICON2POSE: Publish error - {e}")
                self.pose_buffer.get()
            else:
                break

    def close(self):
        self.on = False
        if self.vicon:
            self.vicon.close()
        if self.subscriber:
            del self.subscriber
        if self.publisher:
            del self.publisher
        if self.session:
            self.session.close()
        while not self.pose_buffer.empty():
            self.pose_buffer.get()
        print("VICON2POSE: Closed")

def main():
    v2p = Vicon2Pose()
    if v2p.on:
        v2p.open()
        try:
            print("Starting VICON2POSE tracking... Press Ctrl+C to stop.")
            while True:
                v2p.loop()
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("Stopping VICON2POSE tracking...")
        finally:
            v2p.close()
    else:
        print("VICON2POSE: Not enabled in config file!")

if __name__ == "__main__":
    main()