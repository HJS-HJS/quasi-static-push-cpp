import csv
import sys
import os
import time
import yaml
import numpy as np
import random
from typing import List

so_file_path = os.path.abspath("../build/build/")
sys.path.append(so_file_path)

from quasi_static_push import SimulationViewer, SimulationResult, SimulationDoneReason, GripperMotion, Player

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
        self.display_size = np.array([self.config["display"]["WIDTH"], self.config["display"]["HEIGHT"] ]) # Get display size parameter from config.yaml

        ## Set parameters
        # Set pixel unit
        self.unit = self.config["display"]["unit"] #[m/pixel]

        # Set pusher
        _pusher_type = self.config["pusher"]["pusher_type"]

        # Set pusher unit speed
        unit_v_speed = self.config["pusher"]["unit_v_speed"] # [m/s]
        unit_r_speed = self.config["pusher"]["unit_r_speed"] # [rad/s]
        unit_w_speed = self.config["pusher"]["unit_w_speed"] # [m/s]
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

        self.simulator = SimulationViewer(
            window_width = self.display_size[0],
            window_height = self.display_size[1],
            scale = 1 / self.config["display"]["unit"],
            gripper_movement = GripperMotion.MOVE_XY,
            headless = not visualize,
            frame_rate = 100.0,
            frame_skip = 10,
            grid = True,
            recording_enabled = False,
            show_closest_point = False,
            )
        
    def reset(self):
            
            # Table setting
            self.table_limit = np.array([1.3, 1.3])

            # Slider setting
            slider_inputs = []
            slider_inputs.append(("ellipse", [0.0  , 0.5 - 0.5, 0.0, 0.12, 0.12]))
            slider_inputs.append(("ellipse", [-0.15, 0.75 - 0.5, 0.0, 0.12, 0.12]))
            slider_inputs.append(("ellipse", [ 0.15, 0.75 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [-0.3 , 1.0 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [ 0.0 , 1.0 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [ 0.3 , 1.0 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [-0.45, 1.25 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [-0.15, 1.25 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [ 0.15, 1.25 - 0.5, 0.0, 0.12, 0.12]))
            # slider_inputs.append(("ellipse", [ 0.45, 1.25 - 0.5, 0.0, 0.12, 0.12]))
            slider_num = len(slider_inputs)
            
            # Initial pusher pose
            self.pusher_input[4] = 0.15    # width
            self.pusher_input[7] = 0.0   # x
            self.pusher_input[8] = -0.25   # y
            self.pusher_input[9] = 0   # w

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
        # print(action)
        
        state_curr = self.simulator.run(action)
        return state_curr.slider_state, state_curr.pusher_state


if __name__=="__main__":
    sim = Simulation(
        # visualize = None
        )
    import time
    sim.reset()

    action = np.array([1.0, 0.0, 0.0, 0.0, 1.0])
    step_count = 0

    csv_filename = "target_positions_mine.csv"
    with open(csv_filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        header = ["step"]
        for i in range(3):
            header += [f"x_{i}", f"y_{i}", f"z_{i}"]
        writer.writerow(header)
    state_slider, state_pusher = sim.step(np.array([0.0, 0.0, 0.0, 0.0, 1.0]))
    time.sleep(3)
    while True:
        state_slider, state_pusher = sim.step(action)
        # time.sleep(0.001)
        if step_count % 10 == 0:
            row = [step_count]
            for pos in state_slider:
                row += [pos[0], pos[1], pos[2]]
            with open(csv_filename, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
        if state_pusher[1] > 0.25:
            break
        step_count += 1
    time.sleep(3)
