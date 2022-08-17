import threading
import time
import cv2

from RobotControl import Serial_Servo_Running as SSR
import running_parameters
import levels.first as first
import levels.door as door
import levels.stairs as stairs
import levels.barrier as barrier
import copy

# import all_together_trap_bridge_stairs_lab_changed as all

global param_data, dim, k, d, scale, p, Knew, map1, map2, color_dict, color_range

# 运行参数 #
cap = cv2.VideoCapture(-1)
Running = True
org_img = None
ret = False
level = 1
debug = True
key = -1
istop = True

# 第一关参数 #
door_count = 0
no_door_count = 0

# 第二关参数 #
door_cnt = 0
door_end = 0


def initialize_parameters():
    global param_data, dim, k, d, scale, p, Knew, map1, map2, color_dict, color_range

    param_data = running_parameters.param_data
    dim = running_parameters.dim
    k = running_parameters.k
    d = running_parameters.d
    scale = running_parameters.scale
    p = running_parameters.p
    Knew = running_parameters.Knew
    map1 = running_parameters.map1
    map2 = running_parameters.map2
    color_dict = running_parameters.color_dict
    color_range = running_parameters.color_range


def get_img():
    global cap, org_img, ret, Running, key
    while True:
        if cap.isOpened():
            ret, org_img_fish = cap.read()
            org_img = cv2.remap(org_img_fish.copy(), map1, map2, interpolation=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)

            if ret:
                cv2.imshow('original frame', org_img)
                time.sleep(0.6)
                key = cv2.waitKey(3)

            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)


def main():
    # 初始化参数
    initialize_parameters()

    # 启动摄像头线程
    th1 = threading.Thread(target=get_img)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th1.start()

    # 启动控制线程
    SSR.start_action_thread()
    time.sleep(3)

    # test第一关，过杆
    first.start_door()

    # flip barrier
    # global door_count, no_door_count
    # barrier.main(org_img=org_img)

    # 过蓝色门
    # global door_cnt, door_end
    # door.find_center_of_next(org_img, door_cnt, door_end)


if __name__ == "__main__":
    main()
