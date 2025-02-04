import sys
import os
import time
import curses

so_file_path = os.path.abspath("build/build")
sys.path.append(so_file_path)  # 정확한 경로 추가

import quasi_static_push

# Python에서 SimulationViewer 객체 생성
viewer = quasi_static_push.SimulationViewer(
    table_size_x = 1.5,
    table_size_y = 1.0,
    show_closest_point = True)

# 시뮬레이션 초기화
viewer.reset()

# u_input으로 시뮬레이션 실행
u_input = [0.001, 0.0, 0.000, 0.005]

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
    start = time.time()
    viewer.run(u_input)
    viewer.render()
    print("Time spent [Hz]: {:.2f}".format(1/(time.time() - start)))
    time.sleep(0.01)  # CPU 부하 방지