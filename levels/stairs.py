import cv2
import threading
import numpy as np
import time

import main
import running_parameters
from levels.necessary_functions import find_center
import RobotControl.Serial_Servo_Running as SSR
from running_parameters import color_dict

cap = cv2.VideoCapture(-1)
org_img = None
ret = False

map1 = running_parameters.map1
map2 = running_parameters.map2


# debug = main.debug
# isstop = main.istop


def get_img():
    global cap, org_img, ret
    while True:
        if cap.isOpened():
            ret, org_img_fish = cap.read()
            org_img = cv2.remap(org_img_fish.copy(), map1, map2, interpolation=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)

            if ret:
                cv2.imshow('original frame', org_img)
                time.sleep(0.6)
                cv2.waitKey(3)

            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)


def main():
    print("stairs")
    time_start_adjust = time.time()

    th1 = threading.Thread(target=get_img)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th1.start()

    while org_img is None:
        time.sleep(0.5)
        print('delay 0.5s')

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