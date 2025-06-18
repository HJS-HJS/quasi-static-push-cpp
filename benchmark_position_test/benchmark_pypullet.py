import csv
import pybullet as p
import pybullet_data
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

time_step = 1. / 120

# PyBullet 연결 (GUI → 시각화, DIRECT → 측정용)
# physicsClient = p.connect(p.DIRECT)
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setTimeStep(time_step)

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
target_pos_list = [
    [ 0.0, 0.5 - 0.5, target_height / 2],
    [-0.15, 0.75 - 0.5, target_height / 2],
    [ 0.15, 0.75 - 0.5, target_height / 2],
    ]
target_ids = []

mass = 1

target_col_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=target_radius, height=target_height)
target_vis_id = p.createVisualShape(p.GEOM_CYLINDER, radius=target_radius, length=target_height, rgbaColor=[1, 0, 0, 1])

for target_pos in target_pos_list:
    target_id = p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=target_col_id,
        baseVisualShapeIndex=target_vis_id,
        basePosition=target_pos
    )
    target_ids.append(target_id)
    # p.changeDynamics(target_id, linkIndex=-1, lateralFriction=0.3)
    p.changeDynamics(target_id, linkIndex=-1, lateralFriction=0.15)
    p.changeDynamics(target_id, linkIndex=-1, restitution=0.0)
    p.changeDynamics(target_id, -1, ccdSweptSphereRadius=0, contactProcessingThreshold=0)

pusher_width = 0.03
pusher_length = 0.06
pusher_height = 0.05
mass = 1.0
quat = R.from_euler("z", np.deg2rad(0.)).as_quat()

pusher_half_extents = [pusher_width, pusher_length, pusher_height]
pusher_pos = np.array([0.0, -0.25, pusher_height / 2])

# 충돌/시각화 shape 생성
pusher_col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=pusher_half_extents)
pusher_center_vis_id = p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=target_height, rgbaColor=[1, 0, 0, 1])
pusher_vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=pusher_half_extents, rgbaColor=[0, 0, 1, 1])

gripper_id = p.loadURDF("pusher.urdf", basePosition=pusher_pos, baseOrientation=[quat[0], quat[1], quat[2], quat[3]])

# p.changeDynamics(gripper_id, linkIndex=-1, lateralFriction=0.3)
p.changeDynamics(gripper_id, linkIndex=-1, lateralFriction=0.7)
p.changeDynamics(gripper_id, linkIndex=-1, restitution=0.0)
p.changeDynamics(gripper_id, -1, ccdSweptSphereRadius=0, contactProcessingThreshold=0)

# gripper movement
velocity = 0.001          # 속도 (m/s)
dt = 0.01                 # 시뮬레이션 타임스텝

def update_gripper(step_count):
    t = step_count * dt
    x = t * velocity + pusher_pos[1]
    pos = [0, x, pusher_pos[2]]  # z 고정
    p.resetBasePositionAndOrientation(gripper_id, pos, quat)

step_count = 0
log_interval = 5  # 초

csv_filename = "target_positions_pybullet.csv"
with open(csv_filename, mode='w', newline='') as f:
    writer = csv.writer(f)
    header = ["step"]
    for i in range(len(target_ids)):
        header += [f"x_{i}", f"y_{i}", f"z_{i}"]
    writer.writerow(header)

# time.sleep(3)
while True:
    update_gripper(step_count)  # gripper 위치 직접 설정
    p.stepSimulation()

    if step_count % 100 == 0:
        row = [step_count]
        for tid in target_ids:
            pos, _ = p.getBasePositionAndOrientation(tid)
            row += [pos[0], pos[1], pos[2]]
        with open(csv_filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)
    
    pos, _ = p.getBasePositionAndOrientation(gripper_id)
    if pos[1] > 0.25:
        break


    step_count += 1

# time.sleep(3)


p.disconnect()
