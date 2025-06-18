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

import quasi_static_push

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
        self.display_size = np.array([1024, 1024]) # Get display size parameter from config.yaml

        ## Set parameters
        # Set pixel unit
        # self.unit = self.config["display"]["unit"] #[m/pixel]
        self.unit = 0.003 #[m/pixel]

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

        self.action_space = np.zeros_like(self.unit_speed)
        self.observation_space = np.zeros(1 + 2 + 4 + 5), np.zeros((2, 5))

        self.simulator = quasi_static_push.SimulationViewer(
            window_width = self.display_size[0],
            window_height = self.display_size[1],
            scale = 1 / self.unit,
            headless = not visualize,
            frame_rate = 100.0,
            frame_skip = 1,
            grid = True,
            recording_enabled = False,
            show_closest_point = False,
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

    def step(self, action):
        if(len(action) != 5): print("Invalid action space")
        
        action = np.clip(action, self.action_limit[:, 0], self.action_limit[:, 1])
        action *= self.unit_speed
        
        self.simulator.run(action)
    
    def generate_spawn_points(self, num_points, center_bias=0.75):
        points = []
        x_range = (-self.table_limit[0] + self.min_r * 1.3, self.table_limit[0] - self.min_r * 1.3)
        y_range = (-self.table_limit[1] + self.min_r * 1.3, self.table_limit[1] - self.min_r * 1.3)

        # 첫 번째 점을 랜덤하게 생성
        center_x = random.uniform(*x_range) * 0.9
        center_y = random.uniform(*y_range) * 0.9
        points.append((center_x, center_y))

        # Raduis of inital point
        init_r = random.uniform(self.min_r, self.max_r)
        available_lengh = (init_r + self.min_r, init_r + self.max_r)
        
        # 나머지 점 생성
        candidate_points = []
        for _ in range(num_points - 1):
            # 첫 번째 점 주변에서 가우시안 분포로 점 생성
            if random.random() < center_bias:  # 중심 근처에 생성될 확률
                new_x = np.clip(np.random.normal(center_x, random.uniform(*available_lengh)), *x_range)
                new_y = np.clip(np.random.normal(center_y, random.uniform(*available_lengh)), *y_range)
            else:  # 전체 영역에 균일 분포로 생성
                new_x = random.uniform(*x_range)
                new_y = random.uniform(*y_range)
            candidate_points.append((new_x, new_y))
        
        # 거리 조건을 만족하는 점만 선택
        for point in candidate_points:
            distances = [np.sqrt((point[0] - p[0])**2 + (point[1] - p[1])**2) for p in points]
            if all(d >= (init_r + self.min_r) for d in distances):
                points.append(point)
        
        points = np.array(points)

        min_distances = np.ones(len(points)) * self.min_r
        min_distances[0] = init_r

        for idx, point in enumerate(points):
            if idx == 0: continue
            distances = [np.sqrt((point[0] - p[0])**2 + (point[1] - p[1])**2) for p in points]
            distances = distances - min_distances
            distances[idx] = self.max_r
            min_distances[idx] = min(distances)

        # 첫 번째 점을 포함한 최종 점 리스트
        return points, np.array(min_distances)

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
        return np.array([vx, vy, omega, 0.0, 1.0])
    
    action = np.array([0.0, 0.0, omega, 0.0, 0.0])
    print("Starting performance test (1 minute real time)...")
    start = time.perf_counter()
    step_count = 0
    log_interval = 5  # 초

    next_log = start + log_interval

    time.sleep(3)
    while True:
        now = time.perf_counter()
        if now - start > 600:
            break
        time.sleep(0.001)
        # sim.simulator.running(action)
        sim.simulator.run(update_gripper(step_count))
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