import pybullet as p
import pybullet_data
import psutil
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

process = psutil.Process(os.getpid())

# PyBullet 연결 (GUI → 시각화, DIRECT → 측정용)
# physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setTimeStep(1. / 240)

p.resetDebugVisualizerCamera(
    cameraDistance=3.0,               # 카메라 거리
    cameraYaw=0,                     # 좌우 회전 (z축 기준)
    cameraPitch=-89.9,                # 위아래 각도 (-90도에 가까우면 top view)
    cameraTargetPosition=[0, 0, 0]    # 카메라가 바라보는 중심
)

# 바닥
p.loadURDF("plane.urdf")

# Table size
table_x = 1.3
table_y = 1.3

# target 파라미터
target_radius = 0.12
target_height = 0.05
target_pos = [-0.2, 1.1, target_height / 2]
# mass = 0.00001
mass = 1

target_col_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=target_radius * 1.2, height=target_height)
target_vis_id = p.createVisualShape(p.GEOM_CYLINDER, radius=target_radius, length=target_height, rgbaColor=[1, 0, 0, 1])

target_id = p.createMultiBody(
    baseMass=mass,
    baseCollisionShapeIndex=target_col_id,
    baseVisualShapeIndex=target_vis_id,
    basePosition=target_pos
)

pusher_width = 0.03
pusher_length = 0.06
pusher_height = 0.05
grip_length = 0.15
# grip_length = 0.3
mass = 1.0
quat = R.from_euler("z", np.deg2rad(90.)).as_quat()

pusher_half_extents = [pusher_width, pusher_length, pusher_height]
pusher_pos = np.array([0, 1.1, pusher_height / 2])

# 충돌/시각화 shape 생성
pusher_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=pusher_half_extents)
pusher_center_vis_id = p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=target_height, rgbaColor=[1, 0, 0, 1])
pusher_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=pusher_half_extents, rgbaColor=[0, 0, 1, 1])

gripper_id = p.loadURDF("pusher.urdf", basePosition=pusher_pos, baseOrientation=[quat[0], quat[1], quat[2], quat[3]])

# gripper movement
radius = 1.1                  # 반지름
omega = 0.3               # 각속도 (rad/s)
center = np.array([0, 0]) # 원 중심
dt = 0.01                 # 시뮬레이션 타임스텝

def update_gripper(step_count):
    t = step_count * dt
    x = radius * np.cos(omega * t + np.pi / 2)
    y = radius * np.sin(omega * t + np.pi / 2)
    pos = [x, y, pusher_pos[2]]  # z 고정

    theta = omega * t + np.pi / 2
    quat = R.from_euler("z", theta).as_quat()
    quat = [quat[0], quat[1], quat[2], quat[3]]  # (w, x, y, z)

    p.resetBasePositionAndOrientation(gripper_id, pos, quat)

# ⏱️ 1분 동안 가능한 최대 step 수행
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
    update_gripper(step_count)  # gripper 위치 직접 설정
    p.stepSimulation()
    time.sleep(0.001)
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

p.disconnect()
