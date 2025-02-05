import sys
import os
import time
import numpy as np
import curses
import cv2
so_file_path = os.path.abspath("build/build")
sys.path.append(so_file_path)  # 정확한 경로 추가

import quasi_static_push

# Python에서 SimulationViewer 객체 생성
viewer = quasi_static_push.SimulationViewer(
    # table_size_x = 1.5,
    # table_size_y = 1.0,
    # visualise = False,
    frame_skip = 1,
    recording_enabled = True,
    show_closest_point = False,
    # show_closest_point = True,
    )

# 시뮬레이션 초기화
viewer.reset()

# u_input으로 시뮬레이션 실행

unit_v_speed = 1.0
unit_r_speed = 1.0
unit_w_speed = 1.0

# u_input = [0, 0, 0, 0]  # y, x, rotation, gripper width

# def main(stdscr):
#     curses.cbreak()
#     stdscr.nodelay(True)  # Non-blocking input
#     stdscr.keypad(True)

#     while True:
#         start = time.time()
#         key = stdscr.getch()
        
#         if key == 27:  # ESC key
#             break
#         # Move pusher center in y-axis (ws)
#         elif key == ord('w'): u_input[0] = unit_v_speed / 10  # Move forward
#         elif key == ord('s'): u_input[0] = -unit_v_speed / 10 # Move backward
#         else: u_input[0] = 0  # Stop
        
#         # Move pusher center in x-axis (ad)
#         if key == ord('a'): u_input[1] = unit_v_speed / 10  # Move left
#         elif key == ord('d'): u_input[1] = -unit_v_speed / 10 # Move right
#         else: u_input[1] = 0  # Stop
        
#         # Rotate pusher center (qe)
#         if key == ord('q'): u_input[2] = unit_r_speed / 10  # Turn ccw
#         elif key == ord('e'): u_input[2] = -unit_r_speed / 10 # Turn cw
#         else: u_input[2] = 0  # Stop
        
#         # Control gripper width (left, right)
#         if key == curses.KEY_LEFT:  u_input[3] = -unit_w_speed  # Left arrow
#         elif key == curses.KEY_RIGHT: u_input[3] = unit_w_speed   # Right arrow
#         else: u_input[3] = 0  # Stop
        
#         viewer.run(u_input)
#         viewer.render()
#         print("Time spent [Hz]: {:.2f}".format(1/(time.time() - start)))
#         time.sleep(0.01)  # CPU 부하 방지

# # 실행
# curses.wrapper(main)

while True:
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
    
    viewer.reset(
        slider_inputs = slider_inputs,
        pusher_input = pusher_input,
        newtableWidth = newtableWidth,
        newtableHeight = newtableHeight
    )

    u_input = [0.5, 0.0, 0.000, 0.1, 0]
    for i in range(1000):
        if i > 100:
            u_input[4] = 1
            time.sleep(0.1)
        start = time.time()
        result = viewer.run(u_input)
        print("Time spent [Hz]: {:.2f}".format(1/(time.time() - start)))

        time.sleep(0.0001)  # CPU 부하 방지
        if result.done: 
            print(result.reasons)
            time.sleep(1)
            break
    print("finish")
    

# int done;
# std::vector<std::string> reasons;
# py::object image_state;
# int mode;
# py::object pusher_state;
# py::object slider_state;