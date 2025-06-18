import psutil
import sys
import os
import time
import yaml
import numpy as np
import random
from typing import List

so_file_path = os.path.abspath("../build/build/")
sys.path.append(so_file_path)

import quasi_static_simple2

process = psutil.Process(os.getpid())

class Simulation():
    def __init__(self, visualize:str = 'human'):
        # Set display visuality
        if visualize == "human": visualize = True
        elif visualize is None: visualize = False
        else: visualize = False

        ## Get config file
        with open(os.path.dirname(os.path.abspath(__file__)) + "/config.yaml") as f:
            self.config = yaml.load(f,Loader=yaml.FullLoader)

        # Set patameters
        # self.display_size = np.array([320, 240]) # Get display size parameter from config.yaml
        self.display_size = np.array([1280, 960]) # Get display size parameter from config.yaml

        ## Set parameters
        # Set pixel unit
        # self.unit = 0.002929688 #[m/pixel]
        self.unit = 0.005 #[m/pixel]

        # Set pusher
        _pusher_type = self.config["pusher"]["pusher_type"]

        # Set pusher unit speed
        unit_v_speed = self.config["pusher"]["unit_v_speed"] / 2 # [m/s]
        unit_r_speed = self.config["pusher"]["unit_r_speed"] * 2  # [rad/s]
        unit_w_speed = self.config["pusher"]["unit_w_speed"] * 2  # [m/s]
        self.unit_speed = [unit_v_speed, unit_v_speed, unit_r_speed, unit_w_speed, int(1)]

        # Set slider 
        self.slider_max_num = 1 # Get sliders number
        self.min_r = self.config["auto"]["minimum_radius"]
        self.max_r = self.config["auto"]["maximum_radius"]

        self.pusher_input = [
            self.config["pusher"]["pusher_num"], self.config["pusher"]["pusher_angle"], _pusher_type["type"], 
            {"a": _pusher_type["a"], "b": _pusher_type["b"], "n": _pusher_type["n"]}, 
            self.config["pusher"]["pusher_distance"], self.config["pusher"]["pusher_d_u_limit"], self.config["pusher"]["pusher_d_l_limit"],
            0.0, 0.0, 0.0
        ]

        self.action_limit = np.array([
            [-1., 1.],
            [-1., 1.],
            [-1., 1.],
            [-1., 1.],
            [0, 1],
        ])

        self.simulator = quasi_static_simple2.SimpleSimulationViewer2(
            window_width = self.display_size[0],
            window_height = self.display_size[1],
            scale = 1 / self.unit,
            tableWidth = 3.0,
            tableHeight = 3.0,
            frame_rate = 100,
            headless = visualize,
            )
        
    def reset(self):
        # target 파라미터
        target_radius = 0.14
        target_pos = [-0.2, 1.1]

        # Table setting
        _table_limit_width  = int(2.6 / self.unit)
        _table_limit_height = int(2.6 / self.unit)
        _table_limit = np.array([_table_limit_width, _table_limit_height])
        self.table_limit = _table_limit * self.unit / 2

        # Slider setting
        slider_inputs = []
        slider_inputs.append(("ellipse", [target_pos[0], target_pos[1], 0.0, target_radius, target_radius]))
        
        # Initial pusher pose
        self.pusher_input[4] = 0.185    # width
        self.pusher_input[7] = 0   # x
        self.pusher_input[8] = 1.1   # y
        self.pusher_input[9] = np.deg2rad(90)   # w
        
        self.simulator.reset(
            slider_inputs = slider_inputs,
            pusher_input = tuple(self.pusher_input),
            newtableWidth = self.table_limit[0] * 2,
            newtableHeight = self.table_limit[1] * 2,
        )

        return

if __name__=="__main__":
    print("init")
    sim = Simulation(
        # visualize = None
        )
    import time
    print("reset")
    sim.reset()

    # gripper movement
    radius = 1.1                  # 반지름
    omega = 0.3               # 각속도 (rad/s)
    center = np.array([0, 0]) # 원 중심
    dt = 0.01                 # 시뮬레이션 타임스텝

    def update_gripper(step_count):
        theta = omega * step_count * dt + np.pi / 2
        vx = radius * omega * np.cos(theta)
        vy = radius * omega * np.sin(theta)
        return np.array([vx, vy, omega, 0.0, 0.0])
    
    action = np.array([0.0, 0.0, omega, 0.0, 0.0])
    print("Starting performance test (1 minute real time)...")
    start = time.perf_counter()
    step_count = 0
    log_interval = 5  # 초

    next_log = start + log_interval

    while True:
        now = time.perf_counter()
        if now - start > 600:
            break
        # sim.simulator.running(action)
        sim.simulator.running(update_gripper(step_count))
        step_count += 1

        if now >= next_log:
            cpu = process.cpu_percent(interval=None)
            mem = process.memory_info().rss / 1024 / 1024
            print(f"[{now - start:.1f}s] CPU: {cpu:.1f}% | Mem: {mem:.1f} MB | Steps: {step_count}")
            next_log += log_interval

    elapsed = time.perf_counter() - start


    print(f"Total steps: {step_count}")
    print(f"Elapsed time: {elapsed:.2f} sec")
    print(f"Effective FPS: {step_count / elapsed:.2f} steps/sec")