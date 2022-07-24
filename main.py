import threading
import time
import cv2
import copy

from RobotControl import Serial_Servo_Running as SSR
import running_parameters
import levels.door as door

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
    global cap, org_img, ret, Running, debug, key
    while True:
        if cap.isOpened():
            ret, org_img_fish = cap.read()
            org_img = cv2.remap(org_img_fish.copy(), map1, map2, interpolation=cv2.INTER_LINEAR,
                                borderMode=cv2.BORDER_CONSTANT)

            if ret:
                if not debug:
                    cv2.imshow('original frame', org_img)
                    # time.sleep(0.5)
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

    # 第一关
    door.start_door(width=320, height=240, )


if __name__ == "__main__":
    main()
