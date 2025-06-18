import psutil
import mujoco
from mujoco import MjModel, MjData
from mujoco.viewer import launch_passive
import numpy as np
import tempfile
import os
import time
from scipy.spatial.transform import Rotation as R

process = psutil.Process(os.getpid())

# Define MuJoCo XML environment
mjcf = """
<mujoco>
    <option timestep="0.01"/>
    <worldbody>
        <!-- Table -->
        <geom name="table" type="plane" pos="0 0 0" size="1.3 1.3 0.1" rgba="0.8 0.8 0.8 1" mass="0.00001"/>
        <!-- Base cylinder -->
        <body name="base_cylinder" pos="-0.2 1.1 0.05">
            <geom type="cylinder" size="0.12 0.05" rgba="1 0 0 1"/>
            <joint type="free"/>
        </body>
        <!-- 중심 + 팔을 하나의 body로 묶음 -->
        <body name="gripper" pos="0 1.1 0.05" euler="0 0 90">
            <joint name="gripper_free_joint" type="free"/>
            
            <!-- 중심 푸셔 geom -->
            <geom type="cylinder" size="0.03 0.05" rgba="0.2 0.2 1 1" mass="10"/>

            <!-- Arm1: 120도 -->
            <geom pos="0.1602147 0.0925 0" euler="0 0 120"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>

            <!-- Arm2: 240도 -->
            <geom pos="-0.1602147 0.0925 0" euler="0 0 240"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>

            <!-- Arm3: 0도 -->
            <geom pos="0 -0.185 0" euler="0 0 0"
                    type="box" size="0.015 0.03 0.05" rgba="1 0 0 1" mass="10"/>
        </body>
    </worldbody>

    <actuator>
        <!-- Add actuators if needed for control -->
    </actuator>
</mujoco>
"""

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
radius = 1.1                  # 반지름
omega = 0.3               # 각속도 (rad/s)
center = np.array([0, 0]) # 원 중심
dt = 0.01                 # 시뮬레이션 타임스텝

print("Starting performance test (1 minute real time)...")
start = time.perf_counter()
step_count = 0
log_interval = 5  # 초

next_log = start + log_interval

qpos_adr = model.jnt_qposadr[model.joint("gripper_free_joint").id]


time.sleep(3)
while True:
    now = time.perf_counter()
    if now - start > 600:
        break
    t = step_count * dt

    # 속도 계산
    theta = omega * t + np.pi / 2
    x =  radius * np.cos(theta)
    y =  radius * np.sin(theta)
    quat = R.from_euler("z", theta).as_quat()

    time.sleep(0.001)
    # gripper의 자유 joint 속도 설정
    data.qpos[qpos_adr + 0: qpos_adr + 2] = [x, y]
    data.qpos[qpos_adr + 3: qpos_adr + 7] = [quat[3], quat[0], quat[1], quat[2]]

    # 회전은 필요 없다면 angular 속도는 0

    mujoco.mj_forward(model, data)
    mujoco.mj_step(model, data)
    viewer.sync()
    step_count += 1

    if now >= next_log:
        cpu = process.cpu_percent(interval=None)
        mem = process.memory_info().rss / 1024 / 1024
        print(f"[{now - start:.1f}s] CPU: {cpu:.1f}% | Mem: {mem:.1f} MB | Steps: {step_count}")
        next_log += log_interval

elapsed = time.perf_counter() - start
print("\n")
print(f"Total steps: {step_count}")
print(f"Elapsed time: {elapsed:.2f} sec")
print(f"Effective FPS: {step_count / elapsed:.2f} steps/sec")
