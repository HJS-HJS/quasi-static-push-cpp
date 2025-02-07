import sys
import os
import time
import numpy as np
import cv2

so_file_path = os.path.abspath("build/build")
sys.path.append(so_file_path)

from quasi_static_push import SimulationViewer, Player

player = Player("recordings")
for is_new, data, action in player:
    cv2.imshow("Replay", data.image_state)
    time.sleep(1/30)
    if cv2.waitKey(30) == 27:  # ESC 키로 종료
        break
cv2.destroyAllWindows()
del player

# Python에서 SimulationViewer 객체 생성
viewer = SimulationViewer(
    # window_width        = 1600,
    # window_height       = 1600,
    # scale               = 400.0,
    # tableWidth          = 0.5,
    # tableHeight         = 1.0,
    # frame_rate          = 100,
    # frame_skip          = 8,
    grid                = False,
    grid_space          = 0.5,
    headless            = False,
    move_to_target      = True,
    show_closest_point  = False,
    # recording_enabled   = True,
    recording_path      = "recordings",
    )

# 시뮬레이션 초기화
viewer.reset()

slider_inputs = [
    ("circle", [0.0, -0.1, 0.0, 0.13]),
    ("circle", [0.2, 0.2, 0.0, 0.13]),
    ("circle", [-0.2, 0.2, 0.0, 0.13]),
    ("ellipse", [0.0, 0.6, np.random.random(), 0.13, 0.12]),
    ("ellipse", [0.3, 0.6, np.random.random(), 0.13, 0.10]),
    ("ellipse", [-0.3, 0.6, np.random.random(), 0.13, 0.16]),
]

# # 푸셔 입력값 (정수, 실수, 문자열, 딕셔너리, 실수 7개)
pusher_input = (
    3, 120.0, "superellipse", 
    {"a": 0.015, "b": 0.03, "n": 10}, 
    0.10, 0.185, 0.04, 0.0, -0.5, 0.0
)

# 새로운 테이블 크기
newtableWidth = 3 + np.random.random() * 0.5
newtableHeight = 3 + np.random.random() * 0.5

while True:
    viewer.reset(
        slider_inputs = slider_inputs,
        pusher_input = pusher_input,
        newtableWidth = newtableWidth,
        newtableHeight = newtableHeight
    )

    u_input = [0.5, 0.0, 0.000, 0.0, 0]
    episode_start = time.time()
    for i in range(1000):
        if i > 2:
            u_input[4] = 1
        start = time.time()
        input  = viewer.keyboard_input()
        result = viewer.run(input[0])
        print("Time spent [Hz]: {:.2f}".format(1 / (time.time() - start)))

        time.sleep(0.0001)  # CPU 부하 방지
        if input[1]: 
            del viewer
            exit()
        if result.done or input[2]:
            time.sleep(1)
            slider_inputs = []
            for slider in result.slider_state:
                if len(slider) == 4: slider_inputs.append(("circle", [slider[0], slider[1], slider[2], slider[3]]))
                else               : slider_inputs.append(("ellipse",[slider[0], slider[1], slider[2], slider[3], slider[4]]))
            break

    print("\tTime spent [s]: {:.2f}".format((time.time() - episode_start)))