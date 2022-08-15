import cv2
import numpy as np
import time

from levels.necessary_functions import find_center
import RobotControl.Serial_Servo_Running as SSR
from running_parameters import color_dict

def main(org_img):
    print("stairs")
    time_start_adjust = time.time()
    while True:
        find_center(org_img)
        cv2.waitKey(3)
        time_end_adjust = time.time()
        if time_end_adjust - time_start_adjust > 10:
            break
    print("forward")
    SSR.change_action_value("go_middle_stair", 2)
    time.sleep(1)

    # 上台阶
    #     SSR.change_action_value("star_up_one+go", 2)  # 前进
    #     time.sleep(20)
    SSR.change_action_value("star_final", 1)  # 前进
    time.sleep(13)

    SSR.change_action_value("right_move_new_4", 2)
    time.sleep(2)

    SSR.change_action_value("star_final", 1)  # 前进
    time.sleep(13)

    SSR.change_action_value("right_move_new_4", 2)
    time.sleep(4)

    SSR.change_action_value("go_middle_stair", 2)
    time.sleep(2)
    #
    SSR.change_action_value("star_final", 1)  # 前进
    time.sleep(13)

    SSR.change_action_value("right_move_new_4", 2)
    time.sleep(4)
    #
    SSR.change_action_value("1", 1)
    time.sleep(1)

    SSR.change_action_value("star_single_final", 1)
    time.sleep(12)

    # SSR.change_action_value("go_middle_stair", 2)
    # time.sleep(2)

    SSR.change_action_value("turn_right", 1)
    time.sleep(2)

    # SSR.change_action_value("go_middle_change_3", 2)
    # time.sleep(2)

    # 下台阶
    SSR.change_action_value("star_down_single_final", 1)
    time.sleep(10)

    SSR.change_action_value("star_down_single_final", 1)
    time.sleep(10)
    

    SSR.change_action_value("go_middle_stair", 1)
    time.sleep(3)

    SSR.change_action_value("1", 1)
    time.sleep(2)

    # 下坡
    SSR.change_action_value("go_down", 10)
    time.sleep(15)
    
    #
    SSR.change_action_value("go_middle_stair", 1)
    time.sleep(3)
    #

    print('stairs done')