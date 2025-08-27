import time
import vicon_tracker
import numpy as np
import os
import zenoh
import json

class Vicon2Pose:
    def __init__(self):
        self.on = True
        self.zenoh_key = "fdcl/pose_sync"
        self.latency = 0.5
        self.frequency = 5.0
        self.dt_desired = 0.2
        self.R_sv = np.array([[0.0, -1.0, 0.0],
                              [1.0,  0.0, 0.0],
                              [0.0,  0.0, 1.0]])
        self.x_v = np.zeros(3)
        self.R_vm = np.eye(3)
        self.x_pose_sync = np.zeros(3)
        self.R_pose_sync = np.eye(3)
        self.timestamp = 0.0
        self.last_publish_time = time.time()
        self.vicon = None
        self.session = None
        self.publisher = None
        self.load_config("../config.cfg")

    def load_config(self, config_file):
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
            print(f"VICON2POSE: Loaded config - object: {self.object_name}, zenoh_key: {self.zenoh_key}, "
                  f"latency: {self.latency}s, frequency: {self.frequency}Hz, dt_desired: {self.dt_desired}s")
        except FileNotFoundError:
            print(f"VICON2POSE: Config file {config_file} not found, using defaults")
            self.object_name = "OriginsX@192.168.10.1"
        except Exception as e:
            print(f"VICON2POSE: Error reading config file: {e}, using defaults")
            self.object_name = "OriginsX@192.168.10.1"

    def open(self):
        try:
            self.vicon = vicon_tracker.vicon()
            self.vicon.open(self.object_name)
            conf = zenoh.Config()
            self.session = zenoh.open(conf)
            self.publisher = self.session.declare_publisher(self.zenoh_key)
            print(f"VICON2POSE: Opened Vicon tracker for {self.object_name} and Zenoh publisher on {self.zenoh_key}")
        except Exception as e:
            print(f"VICON2POSE: Failed to open - {e}")
            self.on = False

    def loop(self):
        if not self.on:
            return
        x_v, R_vm = self.vicon.loop()
        self.x_v = np.array(x_v)
        self.R_vm = np.array(R_vm)
        self.x_pose_sync = self.R_sv @ self.x_v
        self.R_pose_sync = self.R_sv @ self.R_vm
        self.timestamp = time.time() + self.latency

        current_time = time.time()
        if current_time - self.last_publish_time >= self.dt_desired:
            pose_data = [[float(self.R_pose_sync[i, j]) for j in range(3)] + [float(self.x_pose_sync[i])] for i in range(3)]
            payload = {"image_taken_time": self.timestamp, "pose": pose_data}
            try:
                self.publisher.put(json.dumps(payload))
                self.last_publish_time = current_time
                print(f"VICON2POSE: Published - t: {self.timestamp}, x_pose_sync: {self.x_pose_sync}, "
                      f"R_pose_sync:\n{self.R_pose_sync}")
            except Exception as e:
                print(f"VICON2POSE: Publish error - {e}")

    def close(self):
        self.on = False
        if self.vicon:
            self.vicon.close()
        if self.publisher:
            self.publisher.undeclare()
        if self.session:
            self.session.close()
        print("VICON2POSE: Closed")

def main():
    v2p = Vicon2Pose()
    if v2p.on:
        v2p.open()
        try:
            print("Starting VICON2POSE tracking... Press Ctrl+C to stop.")
            while True:
                v2p.loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Stopping VICON2POSE tracking...")
        finally:
            v2p.close()
    else:
        print("VICON2POSE: Not enabled in config file!")

if __name__ == "__main__":
    main()