import threading
import cv2
import numpy as np
import time

import main
import running_parameters
from levels.necessary_functions import getAreaMaxContour
import RobotControl.Serial_Servo_Running as SSR
from running_parameters import color_dict

cap = cv2.VideoCapture(0)
org_img = None
ret = False

map1 = running_parameters.map1
map2 = running_parameters.map2

debug = main.debug
isstop = main.istop


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


# 第1关：起点
def start_door(width=320, height=240):
    global org_img, debug, isstop

    th1 = threading.Thread(target=get_img)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th1.start()

    SSR.serial_setServo(20, 450, 500)
    time.sleep(1)
    door_count = 0
    no_door_count = 0
    print('进入start_door')
    while True:
        border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        frame_door1 = cv2.inRange(frame_hsv, color_dict['yellow_door']['Lower'],
                                  color_dict['yellow_door']['Upper'])  # 对原图像和掩模(颜色的字典)进行位运算
        frame_door2 = cv2.inRange(frame_hsv, color_dict['black']['Lower'],
                                  color_dict['black']['Upper'])  # 对原图像和掩模(颜色的字典)进行位运算
        frame_door = cv2.add(frame_door1, frame_door2)
        opened = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours, 25)  # 找出最大轮廓
        percent = round(100 * area_max / (width * height), 2)  # 最大轮廓的百分比

        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
            box = np.int32(cv2.boxPoints(rect))  # 点的坐标
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
        if debug:
            #             cv2.imshow('orginal frame', org_img)
            #             cv2.imshow('closed', closed)  # 显示图像
            cv2.putText(org_img_copy, 'area: ' + str(percent) + '%', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)
            # cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0,
            # 255), 2)
        #             cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
        # cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
        # cv2.waitKey(3)

        # 根据比例得到是否前进的信息
        if percent > 20:
            print(percent)
            print('stop')
            isstop = True
            # SSR.change_action_value("1",10)  # 静止
            time.sleep(0.1)
            door_count += 1
            no_door_count = 0
        elif door_count >= 10:
            no_door_count += 1
            if no_door_count >= 10:
                isstop = False
                print('gogogo')
                print(percent)
                SSR.change_action_value("go_middle_stair", 5)  # gogogo
                time.sleep(6)
                return debug, isstop
        else:
            pass
        print("door_count = ", door_count, " no_door_count = ", no_door_count)

#         cv2.destroyAllWindows()
#         time.sleep(0.5)
