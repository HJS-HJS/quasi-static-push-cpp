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
        self.display_size = np.array([320, 240]) # Get display size parameter from config.yaml

        ## Set parameters
        # Set pixel unit
        self.unit = self.config["display"]["unit"] #[m/pixel]

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
            # self.config["pusher"]["pusher_num"], self.config["pusher"]["pusher_angle"], _pusher_type["type"], 
            # {"a": _pusher_type["a"], "b": _pusher_type["b"], "n": _pusher_type["n"]}, 
            2, 180, "ellipse", 
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
            scale = 1 / self.config["display"]["unit"],
            headless = not visualize,
            frame_rate = 20000.0,
            frame_skip = 1,
            grid = False,
            recording_enabled = False,
            show_closest_point = False,
            )
        
    def reset(self, 
              table_size:List[float] = None,
              slider_inputs:List[List[float]] = None,
              slider_num:int=None,
              ):
            
            # Table setting
            if table_size is None:
                _table_limit_width  = random.randint(int(self.display_size[0] * 0.36), int(self.display_size[0] * 0.86))
                _table_limit_height = random.randint(int(self.display_size[1] * 0.36), int(self.display_size[1] * 0.86))
                _table_limit = np.array([_table_limit_width, _table_limit_height])
                table_size = _table_limit * self.unit
            else:
                _table_limit = (np.array(table_size) / self.unit).astype(int)
            self.table_limit = _table_limit * self.unit / 2

            # Slider setting
            if slider_inputs is None:
                slider_inputs = []
                _slider_num = random.randint(1, self.slider_max_num) if slider_num is None else np.clip(slider_num, 1, 15)
                points, radius = self.generate_spawn_points(_slider_num)
                for point, _r in zip(points, radius):
                    a = np.clip(random.uniform(0.8, 1.0) * _r, a_min=self.min_r, a_max=_r)
                    b = np.clip(random.uniform(0.75, 1.25) * a, a_min=self.min_r, a_max=_r)
                    r = random.uniform(0, np.pi * 2)
                    slider_inputs.append(("ellipse", [point[0], point[1], r, a, b]))
            slider_num = len(slider_inputs)
            
            # Initial pusher pose
            self.pusher_input[4] = random.uniform(self.pusher_input[5], self.pusher_input[6])    # width
            self.pusher_input[7] = 0   # x
            self.pusher_input[8] = 0   # y
            self.pusher_input[9] = 0   # w

            self.simulator.reset(
                slider_inputs = slider_inputs,
                pusher_input = tuple(self.pusher_input),
                newtableWidth = self.table_limit[0] * 2,
                newtableHeight = self.table_limit[1] * 2,
            )

            return None, {None, None, None}

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
    sim = Simulation(
        # visualize = None
        )
    import time
    slider_num = 1
    sim.reset(slider_num=slider_num)
    print(slider_num)

    action = np.array([0.0, 0.0, 0.1, 0.0, 0.0])
    print("Starting performance test (1 minute real time)...")
    start = time.perf_counter()
    step_count = 0
    log_interval = 5  # 초

    next_log = start + log_interval

    while True:
        now = time.perf_counter()
        if now - start > 60:
            break
        state = sim.simulator.run(action)
        time.sleep(0.1)
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