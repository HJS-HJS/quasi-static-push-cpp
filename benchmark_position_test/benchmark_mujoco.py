import csv
import mujoco
from mujoco import MjModel, MjData
from mujoco.viewer import launch_passive
import numpy as np
import tempfile
import os
import time
from scipy.spatial.transform import Rotation as R

# Define MuJoCo XML environment
mjcf = """
<mujoco>
    <option timestep="0.01"/>
    <worldbody>
        <!-- Table -->
        <geom name="table" type="plane" pos="0 0 0" size="1.3 1.3 0.1" rgba="0.8 0.8 0.8 1" mass="0.00001"/>
        <!-- Base cylinder -->
        <body name="base_cylinder1" pos="0.0 0.0 0.05">
            <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
            <joint type="free"/>
        </body>
        <body name="base_cylinder2" pos="-0.15 0.25 0.05">
            <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
            <joint type="free"/>
        </body>
        <body name="base_cylinder3" pos=" 0.15 0.25 0.05">
            <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
            <joint type="free"/>
        </body>
        <!-- 중심 + 팔을 하나의 body로 묶음 -->
        <body name="gripper" pos="0 -0.25 0.05" euler="0 0 0">
            <joint name="gripper_free_joint" type="free"/>
            
            <!-- 중심 푸셔 geom -->
            <geom type="cylinder" size="0.03 0.05" rgba="0.2 0.2 1 1" mass="10"/>

            <!-- Arm1: 120도 -->
            <geom pos="0.129903811 0.075 0" euler="0 0 120"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>

            <!-- Arm2: 240도 -->
            <geom pos="-0.129903811 0.075 0" euler="0 0 240"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>

            <!-- Arm3: 0도 -->
            <geom pos="0 -0.15 0" euler="0 0 0"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>
        </body>
    </worldbody>

    <actuator>
        <!-- Add actuators if needed for control -->
    </actuator>
</mujoco>
"""

        # <body name="base_cylinder4" pos="-0.3 0.5 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder5" pos="0.0 0.5 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder6" pos="0.3 0.5 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder7" pos="-0.45 0.75 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder8" pos="-0.15 0.75 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder9" pos="0.15 0.75 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>
        # <body name="base_cylinder10" pos="0.45 0.75 0.05">
        #     <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
        #     <joint type="free"/>
        # </body>

# Save MJCF to a temporary file
with tempfile.NamedTemporaryFile(delete=False, suffix=".xml") as f:
    f.write(mjcf.encode("utf-8"))
    mjcf_path = f.name

# Load model and data
model = MjModel.from_xml_path(mjcf_path)
data = MjData(model)

# Clean up the temp file after loading
os.remove(mjcf_path)

# Launch the MuJoCo viewer

from mujoco.viewer import launch_passive
viewer = launch_passive(model, data)
viewer.cam.azimuth = 90        # camera yaw
viewer.cam.elevation = -90     # top-down
viewer.cam.distance = 5.0
viewer.cam.lookat[:] = [0, 0, 0.05]

# gripper movement
velocity = 0.003          # 속도 (m/s)
dt = 0.01                 # 시뮬레이션 타임스텝

print("Starting performance test (1 minute real time)...")
step_count = 0
log_interval = 5  # 초

qpos_adr = model.jnt_qposadr[model.joint("gripper_free_joint").id]

csv_filename = "target_positions_mujoco.csv"
with open(csv_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    header = ["step"]
    for i in range(3):
        header += [f"x_{i}", f"y_{i}", f"z_{i}"]
    writer.writerow(header)

# Target body 이름
target_names = [f"base_cylinder{i+1}" for i in range(3)]
target_ids = [model.body(name).id for name in target_names]
pusher_name = "gripper"
pusher_id = model.body(pusher_name).id

model.geom_friction[pusher_id] = np.array([0.05, 0.005, 0.0001])
for bid in target_ids:
    model.geom_friction[bid] = np.array([0.05, 0.005, 0.0001])
time.sleep(3)
while True:
    now = time.perf_counter()
    t = step_count * dt

    # 속도 계산
    y = t * velocity -0.5
    quat = R.from_euler("z", 0).as_quat()

    # gripper의 자유 joint 속도 설정
    data.qpos[qpos_adr + 0: qpos_adr + 2] = [0, y]
    data.qpos[qpos_adr + 3: qpos_adr + 7] = [quat[3], quat[0], quat[1], quat[2]]

    # 회전은 필요 없다면 angular 속도는 0
    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    viewer.sync()

    # 매 10 step마다 위치 기록
    if step_count % 100 == 0:
        row = [step_count]
        for bid in target_ids:
            xpos = data.xpos[bid]  # 3D position
            row += [xpos[0], xpos[1], xpos[2]]
        with open(csv_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
        
        xpos = data.xpos[pusher_id]  # 3D position
        row += [xpos[0], xpos[1], xpos[2]]

    if xpos[1] > 0.25:
        break
        
    step_count += 1
time.sleep(3)

