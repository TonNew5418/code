"""
author: Brian Wang
date: 2022.08.15
"""
import cv2
import numpy as np
import time

from levels.necessary_functions import getAreaMaxContour
import RobotControl.Serial_Servo_Running as SSR
from running_parameters import color_range


def main(org_img, width=320, height=240):
    print('-----------start fliping-----------')
    
    SSR.serial_setServo(20, 300, 500)
    
    t1 = cv2.getTickCount()
    border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                value=(255, 255, 255))  # 扩展白边，防止边界无法识别
    org_img_copy = cv2.resize(border, (width, height), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
    frame_barrier = cv2.inRange(frame_hsv, color_range['blue'][0],color_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_barrier, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours, 25)  # 找出最大轮廓
    percent = round(100 * area_max / (width * height), 2)  # 最大轮廓的百分比


    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
        box = np.int32(cv2.boxPoints(rect))  # 点的坐标
        cv2.drawContours(org_img_copy, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上

    SSR.change_action_value("1", 10)  # 静止
    time.sleep(0.1)

    #if percent < 5:
     #   print("percent:", percent)
      #  print('stop')
       # SSR.change_action_value("go_well", 1)  # 前移，顶住墙体
        #time.sleep(1)
        #SSR.change_action_value("1", 10)  # 静止
        #time.sleep(0.1)


        # SSR.serial_setServo(20, 450, 500)
    SSR.change_action_value("0", 10)  # 静止
    time.sleep(2)
    SSR.change_action_value("flip", 1)
    time.sleep(18)
    SSR.change_action_value("back_fast", 5)
    time.sleep(8)
    SSR.change_action_value("turn_left", 4)
    time.sleep(4)
    SSR.change_action_value("left_move_new_8", 2)
    time.sleep(2)


