#!/usr/bin/python3
# -*- coding: utf-8 -*-

import cv2
import cv2 as cv
import numpy as np
import time
import threading
import math
import Serial_Servo_Running as SSR
import signal
import PWMServo
import copy

from CameraCalibration.CalibrationConfig import *

# from cross_trap_mine_2 import go

############################### calibration #######################
# 加载参数
param_data = np.load(calibration_param_path + '.npz')

# 获取参数
dim = tuple(param_data['dim_array'])
k = np.array(param_data['k_array'].tolist())
d = np.array(param_data['d_array'].tolist())

print('加载参数完成')
print('dim:\n', dim)
print('k:\n', k)
print('d:\n', d)

# 截取区域，1表示完全截取
scale = 1
# 优化内参和畸变参数
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)
Knew = p.copy()
if scale:  # change fov
    Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), Knew, dim, cv2.CV_16SC2)

color_range = {'yellow_door': [(10, 43, 46), (34, 255, 255)],
               'red_floor1': [(0, 43, 46), (10, 255, 255)],
               'red_floor2': [(156, 43, 46), (180, 255, 255)],
               'green_bridge': [(35, 43, 20), (100, 255, 255)],
               'yellow_hole': [(10, 70, 46), (34, 255, 255)],
               'black_hole': [(0, 0, 0), (180, 255, 80)],
               'black_gap': [(0, 0, 0), (180, 255, 100)],
               'black_dir': [(0, 0, 0), (180, 255, 46)],
               # 'blue': [(110, 43, 46), (124, 255, 255)],
               'blue': [(95, 43, 90), (130, 255, 255)],
               'black_door': [(0, 0, 0), (180, 255, 46)],
               'white': [(0, 0, 160), (180, 30, 255)]
               }

color_dict = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              # [(110, 43, 46), (124, 255, 255)]
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              'yellow_door': {'Lower': np.array([10, 43, 46]), 'Upper': np.array([34, 255, 255])},
              'black': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 46])},
              'black_obstacle': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 90])},
              'black_dir': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 46])}
              }
lab_range = {
             # 'yellow_door': [(11, 63, 66), (34, 255, 255)],
             # 'black_door': [(0, 0, 0), (80, 255, 5)],
             #
             # 'blue': [(0, 0, 0), (255, 175, 94)], # dark blue
             # 'blue_bridge': [(137, 93, 0), (255, 122, 121)],  # light blue
             #
             # 'red': [(0, 154, 130), (255, 255, 255)],
             #
             # 'green_bridge': [(47, 0, 135), (255, 110, 255)],
             # 'black_hole': [(0, 0, 0), (180, 255, 80)],
             # 'black_gap': [(0, 0, 0), (180, 255, 100)],
             # 'black_dir': [(0, 0, 0), (180, 255, 46)],
             # 'blue_old': [(0, 92, 52), (255, 178, 111)],
             # 'blue_floor' : [(0, 0, 0), (255, 154, 102)],

             # 'white': [(215, 0, 0), (255, 255, 255)],

             'red': [(0, 154, 130), (255, 255, 255)],
             'green': [(47, 0, 125), (255, 110, 255)],
             'blue': [(0, 0, 0), (255, 175, 104)],#94
             'yellow': [(11, 63, 66), (34, 255, 255)],
             'white': [(215, 0, 0), (255, 255, 255)],
             'black': [(0, 0, 0), (50, 255, 255)]

             }
lab_red = [(0, 154, 130), (255, 255, 255)]

########################## 参数设置 ##########################
# video = "orgin_video.mp4"
# cap = cv2.VideoCapture(video)
cap = cv2.VideoCapture(-1)
Running = True
org_img = None  # 全局变量，原始图像
ret = False  # 读取图像标志位

debug = True
key = -1  # waitKey
isstop = True

# level2 parameters
step = 1
# ret = False
move_count = 0
forward_count = 0  # cross mine
end = 0
door_cnt = 0
door_end = 0

r_w = 640
r_h = 480
# should_chose = False
blue_persent = 0


################################################读取图像线程#################################################
def get_img():
    global cap, org_img, Running, debug, key, ret
    while True:
        if cap.isOpened():
            ret, org_img_fish = cap.read()
            # org_img = cv2.remap(org_img_fish.copy(), map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            if ret:
                org_img = cv2.remap(org_img_fish.copy(), map1, map2, interpolation=cv2.INTER_LINEAR,
                                    borderMode=cv2.BORDER_CONSTANT)
                if not debug:
#                     cv2.imshow('orginal frame', org_img)
                    #                     time.sleep(0.5)
                    time.sleep(0.6)
                    key = cv2.waitKey(3)

            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)


# 读取图像线程
th1 = threading.Thread(target=get_img)
th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()

########################## 控制线程 ##########################
SSR.start_action_thread()


# SSR.change_action_value("1",0)  # 静止

########################## 必要函数 ##########################


# 得到最大轮廓和对应的最大面积
def getAreaMaxContour(contours, area_min=4000):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    # area_max_contour = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    for c in contours:  # 遍历所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > area_min:  # 只有在面积大于 area_min 时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓


def getAreaSecondMaxContour(contours, max_area):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    # area_max_contour = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if max_area == contour_area_temp:
            continue
        elif contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 1500:  # 只有在面积大于4000时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓


def getLineSumContour(contours, area=1):
    contours_sum = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点 int0 <=> int64
        edge1 = math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2 = math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio = edge1 / edge2
        if (area_temp > area) and (ratio > 3 or ratio < 0.33):
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        rect = cv2.minAreaRect(c)  # 最小外接矩形
        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点 int0 <=> int64
        edge1 = math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2))
        edge2 = math.sqrt(math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2))
        ratio = edge1 / edge2
        if (area_temp > area) and (ratio > 3 or ratio < 0.33):
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起
    return contours_sum


# 得到全部的总的轮廓
def getSumContour(contours, area=1):
    contours_sum = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if (area_temp > area):
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起
    return contours_sum


def direction_only_angle(resize_width, resize_height):
    global org_img, reset
    angle_flag = False
    angle = 90
    dis = 0
    see = False
    # PWMServo.setServo(1, 1800, 500)
    # PWMServo.setServo(2, 850, 500)
    time.sleep(0.5)
    # SSR.change_action_value("00", 1)  # stand
    while True:
        if reset == 1:  # 跳关或者重置
            cv2.destroyAllWindows()
            break
        OrgFrame = org_img
        frame = cv2.resize(OrgFrame, (resize_width, resize_height), interpolation=cv2.INTER_LINEAR)
        # cv2.imshow('init', frame)
        # 获取图像中心点坐标x, y
        center = []
        # 开始处理图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 3)
        Imask = cv2.inRange(hsv, color_dict['black_dir']['Lower'], color_dict['black_dir']['Upper'])
        Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
#         cv2.imshow('black', Imask)
        _, cnts, hierarchy = cv2.findContours(Imask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出所有轮廓
        cnt_sum = getLineSumContour(cnts, area=1)
        cv2.drawContours(frame, cnt_sum, -1, (255, 0, 255), 3)
        if cnt_sum is not None:
            see = True
            rect = cv2.minAreaRect(cnt_sum)  # 最小外接矩形
            box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
            if math.sqrt(math.pow(box[3, 1] - box[0, 1], 2) + math.pow(box[3, 0] - box[0, 0], 2)) > math.sqrt(
                    math.pow(box[3, 1] - box[2, 1], 2) + math.pow(box[3, 0] - box[2, 0], 2)):
                if box[3, 0] - box[0, 0] == 0:
                    angle = 90
                else:
                    angle = - math.atan((box[3, 1] - box[0, 1]) / (box[3, 0] - box[0, 0])) * 180.0 / math.pi
                if box[3, 1] + box[0, 1] > box[2, 1] + box[1, 1]:
                    edge_y1, edge_y2 = box[3, 1], box[0, 1]
                    dis = int((edge_y1 + edge_y2) / 2)
                    cv2.circle(OrgFrame, (int((box[3, 0] + box[0, 0]) / 2), int((box[3, 1] + box[0, 1]) / 2)), 10,
                               (0, 0, 255), -1)  # 画出中心点
                else:
                    edge_y1, edge_y2 = box[2, 1], box[1, 1]
                    dis = int((edge_y1 + edge_y2) / 2)
                    cv2.circle(OrgFrame, (int((box[2, 0] + box[1, 0]) / 2), int((box[2, 1] + box[1, 1]) / 2)), 10,
                               (0, 0, 255), -1)  # 画出中心点
            else:
                if box[3, 0] - box[2, 0] == 0:
                    angle = 90
                else:
                    angle = - math.atan(
                        (box[3, 1] - box[2, 1]) / (box[3, 0] - box[2, 0])) * 180.0 / math.pi  # 负号是因为坐标原点的问题
                if box[3, 1] + box[2, 1] > box[0, 1] + box[1, 1]:
                    edge_y1, edge_y2 = box[3, 1], box[2, 1]
                    dis = int((edge_y1 + edge_y2) / 2)
                    cv2.circle(OrgFrame, (int((box[3, 0] + box[2, 0]) / 2), int((box[3, 1] + box[2, 1]) / 2)), 10,
                               (0, 0, 255), -1)  # 画出中心点
                else:
                    edge_y1, edge_y2 = box[0, 1], box[1, 1]
                    dis = int((edge_y1 + edge_y2) / 2)
                    cv2.circle(OrgFrame, (int((box[0, 0] + box[1, 0]) / 2), int((box[0, 1] + box[1, 1]) / 2)), 10,
                               (0, 0, 255), -1)  # 画出中心点
        else:
            see = False

        cv2.putText(frame, "angle:" + str(angle),
                    (10, frame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

#         cv2.imshow('frame', frame)
#         cv2.waitKey(1)

        if not see:  # not see the edge
            cv2.destroyAllWindows()
            break
        else:
            print(dis, angle)
            if dis < 80:
                if angle > -13:
                    # SSR.change_action_value("202", 1)  # 左转小
                    print("left angle small")
                elif angle < -16:
                    # SSR.change_action_value("203", 1)  # 右转小
                    print("right angle small")
                else:
                    break
            elif dis > 400:
                if angle > -24:
                    # SSR.change_action_value("202", 1)  # 左转小
                    print("left angle small")
                elif angle < -28:
                    # SSR.change_action_value("203", 1)  # 右转小
                    print("right angle small")
                else:
                    break
            else:
                if angle > -18:
                    # SSR.change_action_value("202", 1)  # 左转小
                    print("left angle small")
                elif angle < -22:
                    # SSR.change_action_value("203", 1)  # 右转小
                    print("right angle small")
                else:
                    break
            time.sleep(0.2)


def judge_color():
    global org_img
    color = 'blue'  # blue
    border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                value=(255, 255, 255))  # 扩展白边，防止边界无法识别
    org_img_copy = cv2.resize(org_img, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    # frame_temp_1=cv2.inRange(frame_hsv, lab_range['blue'][0], lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    frame_green = cv2.inRange(frame_hsv, lab_range['green'][0],
                              lab_range['green'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened_green = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed_green = cv2.morphologyEx(opened_green, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result_green = cv2.bitwise_and(rgb_image, rgb_image, mask=closed_green)
    frame_blue = cv2.inRange(frame_hsv, lab_range['blue'][0], lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened_blue = cv2.morphologyEx(frame_blue, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed_blue = cv2.morphologyEx(opened_blue, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    # rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result_blue = cv2.bitwise_and(rgb_image, rgb_image, mask=closed_blue)
    (image, contours_blue, hierarchy) = cv2.findContours(closed_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    (image, contours_green, hierarchy) = cv2.findContours(closed_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    areaMaxContour, area_max_blue = getAreaMaxContour(contours_blue)
    areaMaxContour, area_max_green = getAreaMaxContour(contours_green)
    if area_max_blue > area_max_green:
        color = 'blue'
    else:
        color = 'green'
    return color


def check_stick():
    global org_img
    org_img_copy = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_door = cv2.inRange(frame_hsv[20:120, 0:640], lab_range['blue'][0],
                             lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened_door = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones(( ), np.uint8))  # 开运算 去噪点
    closed_door = cv2.morphologyEx(opened_door, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    (image_door, contours_door, hierarchy_door) = cv2.findContours(closed_door, cv2.RETR_LIST,
                                                                   cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    cv2.imshow('check', closed_door)
    areaMaxContour_door_1, area_max_door = getAreaMaxContour(contours_door, 1000)
    if areaMaxContour_door_1 is None:
        print('stick done with no stick')
        SSR.change_action_value("go_middle_change_3", 5)  # gogogo
        time.sleep(10)
        return
    left_count = 0
    SSR.serial_setServo(19, 600, 500)
    time.sleep(3)
    org_img_copy = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_door = cv2.inRange(frame_hsv[20:120, 0:640], lab_range['blue'][0],
                             lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened_door = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones(( ), np.uint8))  # 开运算 去噪点
    closed_door = cv2.morphologyEx(opened_door, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    (image_door, contours_door, hierarchy_door) = cv2.findContours(closed_door, cv2.RETR_LIST,
                                                                   cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    cv2.imshow('check', closed_door)
    SSR.serial_setServo(19, 500, 500)
    time.sleep(1)
    if contours_door is not None:
        areaMaxContour_door_1, area_max_door = getAreaMaxContour(contours_door, 500)
        if areaMaxContour_door_1 is not None:
            if len(contours_door) > 1:
                areaMaxContour_door_2, area_max_door = getAreaMaxContour(contours_door, 500)
                if areaMaxContour_door_2 is not None:
                    print('get to left')
                    SSR.change_action_value("left_move_new_8", 1)
                    time.sleep(1)
                    left_count = 1
                    return
    if left_count == 0:
        SSR.serial_setServo(19, 400, 500)
        time.sleep(3)
        org_img_copy = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
        frame_door = cv2.inRange(frame_hsv[20:120, 0:640], lab_range['blue'][0],
                                 lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened_door = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones(( ), np.uint8))  # 开运算 去噪点
        closed_door = cv2.morphologyEx(opened_door, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        (image_door, contours_door, hierarchy_door) = cv2.findContours(closed_door, cv2.RETR_LIST,
                                                                       cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        SSR.serial_setServo(19, 500, 500)
        time.sleep(1)
        if contours_door is not None:
            areaMaxContour_door_1, area_max_door = getAreaMaxContour(contours_door, 300)
            if areaMaxContour_door_1 is not None:
                if len(contours_door) > 1:
                    areaMaxContour_door_2, area_max_door = getAreaMaxContour(contours_door, 300)
                    if areaMaxContour_door_2 is not None:
                        print('get to right')
                        SSR.change_action_value("right_move_new_4", 1)
                        time.sleep(0.8)
                        left_count = 1
                        return
        print('stick done ')
        SSR.change_action_value("go_middle_change_3", 10)  # gogogo
        time.sleep(10)


########################## 关卡 ##########################

# 第1关：起点
def start_door(width, height):
    global org_img, debug, isstop
#     SSR.serial_setServo(20, 450, 500)
    SSR.serial_setServo(20, 450, 500)
    time.sleep(1)
    door_count = 0
    no_door_count = 0
    print('进入start_door')
    while True:
        t1 = cv2.getTickCount()
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
        _, contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
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
            cv2.putText(org_img_copy, 'area: ' + str(percent) + '%', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            # cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
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
                break
        else:
            pass
        print("door_count = ", door_count, " no_door_count = ", no_door_count)


#         cv2.destroyAllWindows()
#         time.sleep(0.5)



# 第2关：过坑
def cross_trap_all():
    if end == 0:
        SSR.change_action_value("left_move_new_6", 2)  # 左移
        print("left")
        time.sleep(1.2)
        SSR.serial_setServo(20, 340, 500)
        time.sleep(0.5)
        SSR.serial_setServo(19, 500, 500)
        time.sleep(0.5)
        go()


#         pass
# go()
def cross_trap():
    global org_img, step, reset, end

    # initialize
    straight = False
    left = False
    left2 = False
    right = False
    right2 = False

    # border = cv2.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
    #                             value=(255, 255, 255))  # 扩展白边，防止边界无法识别
    org_img_copy = cv2.resize(org_img[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    color = judge_color()
    print(color)
    # frame_temp_1=cv2.inRange(frame_hsv, lab_range['blue'][0], lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    frame_green = cv2.inRange(frame_hsv, lab_range[color][0], lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    # print(closed)
    cv2.imshow('closed', closed)

    (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像

    areaMaxContour, area_max = getAreaMaxContour(contours)
    image_line_detection = np.zeros(closed.shape, np.uint8)

    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    lines = cv2.HoughLinesP(image_line_detection, 1, np.pi / 180, 30, 300, 5)

    ################################调整转向部分##############################

    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y1 < 410 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    tangent_list.append([x1, y1, x2, y2])
        if len(tangent_list) > 0:
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print(tangent_used)

        cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 1)
        if 0.20 > tangent_used > 0.07:
            print("should turn left little")
            SSR.change_action_value("turn_left", 1)
            time.sleep(0.7)
        elif tangent_used > 0.20:
            print("should turn left large")
            SSR.change_action_value("turn_left", 1)
            time.sleep(0.7)
        elif -0.07 > tangent_used > -0.20:
            print("should turn right little")
            SSR.change_action_value("turn_right", 1)
            time.sleep(0.7)
        elif tangent_used < - 0.20:
            print("should turn right large")
            SSR.change_action_value("turn_right", 1)
            time.sleep(0.7)
        print(y1_f, y2_f, area_max)
        # cv2.imshow("image_line1", image_copy)
        if y1_f > 340 and y2_f > 340 and area_max < 45000:
            print("get end")
            time.sleep(1)
            SSR.change_action_value("go_middle_change_3", 3)  # 前进
            time.sleep(2)
            SSR.change_action_value("right_move_new_2", 5)  # 右移
            time.sleep(15)
            SSR.change_action_value("turn_right", 1)
            time.sleep(3)
            SSR.change_action_value("go_middle_change_3", 3)  # 前进
            time.sleep(2)
            end = 1
            return

    ##########################开始避障检测

    if areaMaxContour is not None:
        cv2.fillPoly(image_filled, pts=[areaMaxContour], color=0)
        if len(contours) > 1:
            contours.remove(areaMaxContour)
            areaMaxContour, area_max = getAreaMaxContour(contours, 4000)
            # print(area_max)
            if area_max > 4000:
                # pass
                # print(areaMaxContour)
                cv2.fillPoly(image_filled, pts=[areaMaxContour], color=255)
    else:
        print('trap_end')
        time.sleep(1)
        SSR.change_action_value("go_middle_change_3", 3)  # 前进
        time.sleep(2)
        SSR.change_action_value("right_move_new_2", 3)  # 右移
        time.sleep(8)
        #SSR.change_action_value("turn_right", 1)
        #time.sleep(3)
        SSR.change_action_value("go_middle_change_3", 1)  # 前进
        time.sleep(2)
        end = 1
        return

    image_line_detection = image_line_detection[15:465, 20:620]
    image_line_detection = cv2.copyMakeBorder(image_line_detection, 15, 15, 20, 20, borderType=cv2.BORDER_CONSTANT,
                                              value=0)

    Imask = image_filled

    n = -1
    obscle_list = []
    obscle_list_last = []
    for r in roi:
        n += 1
        blobs = Imask[r[0]:r[1], r[2]:r[3]]
        _, cnts, hierarchy = cv2.findContours(blobs, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt_large, _ = getAreaMaxContour(cnts, 200)
        #####################################change 80 to 200##########################
        if cnt_large is not None:
            # print(area_max)
            lefttop_x = r[2]
            lefttop_y = r[0]
            rightbottom_x = r[3]
            rightbottom_y = r[1]
            cv2.rectangle(image_copy, (lefttop_x, lefttop_y), (rightbottom_x, rightbottom_y), (0, 255, 255), 2)
            # cv2.imshow("what",org_img)
            obscle_list.append([n // 9, n % 9])

    s_straight = [[5, 4], [4, 4], [5, 3], [5, 5]]
    # s_straight = [[5, 4], [4, 4]]
    s_left_edge = [[4, 0], [3, 0], [2, 0], [1, 0]]
    s_right_edge = [[4, 8], [3, 8], [2, 8], [1, 8]]
    s_left_1 = [[4, 1], [3, 1], [2, 1], [4, 2]]
    s_left_2 = [[3, 2], [2, 2], [1, 3], [2, 3]]

    s_right_1 = [[4, 7], [3, 7], [2, 7], [1, 5]]
    s_right_2 = [[4, 6], [3, 6], [2, 6], [2, 5]]
    s_left_middle_1 = [[4, 4], [4, 3], [5, 4], [5, 3]]
    s_left_middle_2 = [[5, 2], [5, 3], [5, 4]]

    s_right_middle_1 = [[4, 4], [4, 5], [5, 4], [5, 5]]
    s_right_middle_2 = [[5, 4], [5, 5], [5, 6]]

    if all(s not in obscle_list for s in s_straight):
        straight = True
        left = False
        left2 = False
        right = False
        right2 = False
    # elif all(s in obscle_list for s in s_edge_left) and any(s in obscle_list for s in s1_2):
    elif all(s in obscle_list for s in s_right_middle_1) or all(s in obscle_list for s in s_right_middle_2):
        straight = False
        left = False
        left2 = True
        right = False
        right2 = False
    elif all(s in obscle_list for s in s_left_middle_1) or all(s in obscle_list for s in s_left_middle_2):
        straight = False
        left = False
        left2 = False
        right = False
        right2 = True
    # elif all(s in obscle_list for s in s_edge_right) and any(s in obscle_list for s in s1_3):

    elif all(s not in obscle_list for s in s_right_1) or all(s not in obscle_list_last for s in s_right_2):
        straight = False
        left = True
        left2 = False
        right = False
        right2 = False
    elif all(s not in obscle_list for s in s_left_1) or all(s not in obscle_list_last for s in s_left_2):
        straight = False
        left = False
        left2 = False
        right = True
        right2 = False
    # elif all(s not in obscle_list for s in s4_1) and all(s not in obscle_list_last for s in s2_1) and any(
    #         s in obscle_list for s in s4_2):
    #     straight = False
    #     left = True
    #     left2 = False
    #     right = False
    #     right2 = False
    # elif all(s not in obscle_list for s in s5_1) and all(s not in obscle_list_last for s in s3_1) and any(
    #         s in obscle_list for s in s5_2):
    #     straight = False
    #     left = False
    #     left2 = False
    #     right = True
    #     right2 = False
    s1 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0], [5, 8]]

    # s1 = [[5, 2], [5, 1],  [5, 6], [5, 7]]
    s1_1 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7]]
    # s1_2 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 8]]
    s1_3 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0]]
    s2_1 = [[5, 1], [5, 0], [5, 3], [5, 2]]
    s2_2 = [[5, 4], [5, 5], [5, 6]]
    s3_1 = [[5, 5], [5, 6], [5, 7], [5, 8]]
    s3_2 = [[5, 4], [5, 3], [5, 2]]
    s4_1 = [[5, 1], [5, 0], [5, 3], [5, 2], [5, 4], [5, 5], [5, 6]]
    s4_2 = [[5, 7], [5, 8]]
    s5_1 = [[5, 5], [5, 6], [5, 7], [5, 8], [5, 3], [5, 2], [5, 4]]
    s5_2 = [[5, 1], [5, 0]]
    s_edge_left = [[4, 0], [3, 0], [2, 0], [1, 0]]
    s_edge_right = [[4, 8], [3, 8], [2, 8], [1, 8]]
    s_adjust = [[4, 4], [4, 3], [4, 2], [4, 1], [4, 5], [4, 6], [4, 7], [4, 0], [4, 8]]
    s_under = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0], [5, 8], [4, 4], [4, 3], [4, 2],
               [4, 1], [4, 5], [4, 6], [4, 7], [4, 0], [3, 8], [3, 4], [3, 3], [3, 2], [3, 1], [3, 5], [3, 6],
               [3, 7], [3, 0], [3, 8]]

    cv2.putText(image_copy,
                "straight:" + str(straight) + " left:" + str(left) + " right:" + str(right) + " left2:" + str(
                    left2) + " right2:" + str(right2),
                (10, org_img.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    cv2.putText(image_copy, "step:" + str(step),
                (10, org_img.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
    # cv2.putText(OrgFrame,"finish:"+str(obscle_finish),(10,OrgFrame.shape[0]-75),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,0,255),2)
    # cv2.imshow('frame', org_img)
    # cv2.waitKey(3)

    #########################正常避障部分 动作组换成move########################

    isgo = False
    count = 0

    if step == 0:
        if all(s not in obscle_list for s in s_under):
            print("straight")
            # SSR.change_action_value("239", 1)  # 前进
            count += 1
            isgo = True
        else:
            if isgo:
                pass
                # SSR.change_action_value('157', 1)

            isgo = False
            step = 1
            time.sleep(0.2)
    elif step == 1:
        if all(s not in obscle_list for s in s_under):
            step = 0
        elif (count >= 20) and any(s in obscle_list for s in s_adjust):
            direction_only_angle(640, 480)
            # PWMServo.setServo(1, 2090, 500)
            # PWMServo.setServo(2, 1480, 500)
            time.sleep(0.8)
            count = 0
        elif straight:
            # SSR.change_action_value("go_forward", 1)  # 前进
            print("forward")
            SSR.change_action_value("go_middle_change_3", 3)  # 前进
            time.sleep(1.5)
            count += 1
        elif left:
            SSR.change_action_value("left_move_new_6", 1)  # 左移
            print("left")
            time.sleep(1)
        elif right:
            SSR.change_action_value("right_move_new_2", 1)  # 右移
            print("right")
            time.sleep(1)
        elif left2 is True:
            SSR.change_action_value("left_move_new_6", 1)  # 左移
            print("left*2")
            time.sleep(1)
        elif right2 is True:
            SSR.change_action_value("right_move_new_2", 1)  # 右移
            print("right*2")
            time.sleep(1)
        else:
            # SSR.change_action_value("1", 1)  # 静止
            print("stand_stand")
            SSR.change_action_value("1", 1)  # 右移

    cv2.imshow("image_org", image_copy)

    # cv2.imshow("image_filled", image_filled)
    # cv2.waitKey(0)
    cv2.imshow("image_line", image_line_detection)

    # cv2.imshow("image1", result)
    cv2.waitKey(3)


def go():
    while True:
        time1 = time.time()
        cross_trap()
        time2 = time.time()
        print("spend", time2 - time1)
        if end:
            cv2.destroyAllWindows()
            return


# lowThreshold = 0
# max_lowThreshold = 100
# ratio = 3
# kernel_size = 3
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# cv2.namedWindow('canny demo')
# cv2.createTrackbar('Min threshold', 'canny demo', lowThreshold, max_lowThreshold, CannyThreshold)
# CannyThreshold(0)  # initialization
# if cv2.waitKey(0) == 27:
#     cv2.destroyAllWindows()


# 第3关：地雷
roi = [(0, 80, 0, 70), (0, 80, 70, 140), (0, 80, 140, 210), (0, 80, 210, 240), (0, 80, 240, 400), (0, 80, 400, 430),
       (0, 80, 430, 500),
       (0, 80, 500, 570), (0, 80, 570, 640), (80, 160, 0, 70), (80, 160, 70, 140), (80, 160, 140, 210),
       (80, 160, 210, 240),
       (80, 160, 240, 400), (80, 160, 400, 430), (80, 160, 430, 500), (80, 160, 500, 570), (80, 160, 570, 640),
       (160, 240, 0, 70),
       (160, 240, 70, 140), (160, 240, 140, 210), (160, 240, 210, 240), (160, 240, 240, 400), (160, 240, 400, 430),
       (160, 240, 430, 500),
       (160, 240, 500, 570), (160, 240, 570, 640), (240, 320, 0, 70), (240, 320, 70, 140), (240, 320, 140, 210),
       (240, 320, 210, 240),
       (240, 350, 240, 400), (240, 320, 400, 430), (240, 320, 430, 500), (240, 320, 500, 570), (240, 320, 570, 640),
       (320, 400, 0, 70), (320, 400, 70, 140), (320, 400, 140, 210), (320, 400, 210, 240), (350, 400, 240, 400),
       (320, 400, 400, 430),
       (320, 400, 430, 500), (320, 400, 500, 570), (320, 400, 570, 640), (400, 430, 0, 70), (400, 430, 70, 140),
       (400, 430, 140, 210),
       (400, 430, 210, 240), (400, 430, 240, 400), (400, 430, 400, 430), (400, 430, 430, 500), (400, 430, 500, 570),
       (400, 430, 570, 640)]
# print(len(roi))
roi_mine = [(0, 80, 0, 70), (0, 80, 70, 140), (0, 80, 140, 210), (0, 80, 210, 280), (0, 80, 280, 360),
            (0, 80, 360, 430),
            (0, 80, 430, 500),
            (0, 80, 500, 570), (0, 80, 570, 640), (80, 160, 0, 70), (80, 160, 70, 140), (80, 160, 140, 210),
            (80, 160, 210, 280),
            (80, 160, 280, 360), (80, 160, 360, 430), (80, 160, 430, 500), (80, 160, 500, 570), (80, 160, 570, 640),
            (160, 240, 0, 70),
            (160, 240, 70, 140), (160, 240, 140, 210), (160, 240, 210, 280), (160, 240, 280, 360), (160, 240, 360, 430),
            (160, 240, 430, 500),
            (160, 240, 500, 570), (160, 240, 570, 640), (240, 320, 0, 70), (240, 320, 70, 140), (240, 320, 140, 210),
            (240, 320, 210, 280),
            (240, 320, 280, 360), (240, 320, 360, 430), (240, 320, 430, 500), (240, 320, 500, 570),
            (240, 320, 570, 640),
            (320, 400, 0, 70), (320, 400, 70, 140), (320, 400, 140, 210), (320, 400, 210, 280), (320, 400, 280, 360),
            (320, 400, 360, 430),
            (320, 400, 430, 500), (320, 400, 500, 570), (320, 400, 570, 640), (400, 480, 0, 70), (400, 480, 70, 140),
            (400, 480, 140, 210),
            (400, 480, 210, 280), (400, 480, 280, 360), (400, 480, 360, 430), (400, 480, 430, 500),
            (400, 480, 500, 570),
            (400, 480, 570, 640)]


def get_angle(color):
    global org_img, blue_persent
    org_img_copy = copy.copy(org_img)
    org_img_copy = cv2.resize(org_img_copy, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_green = cv2.inRange(frame_hsv, lab_range[color][0], lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    #result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    #cv2.imshow('closed', closed)

    (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    # image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像

    areaMaxContour, area_max = getAreaMaxContour(contours, 25)
    center = (1, 1)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)
        center, w_h, angle = rect  # 中心点 宽高 旋转角度
        # box = np.int0(cv2.boxPoints(rect))  # 点的坐标
    image_line_detection = np.zeros(closed.shape, np.uint8)

    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    lines = cv2.HoughLinesP(image_line_detection, 1, np.pi / 180, 30, 300, 5)

    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y1 < 410 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    tangent_list.append([x1, y1, x2, y2])
        if len(tangent_list) > 0:
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print('angle is: ', tangent_used)
        blue_persent = round(area_max * 100 / (r_w * r_h), 2)
        print('percent', blue_persent)
#         cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 3)
#         cv2.circle(image_copy, (int(center[0]), int(center[1])), 5, (255, 0, 0), 2)
#         cv2.imshow("image_line", image_copy)
        return tangent_used


def obscle():
    global org_img, step, reset, move_count, forward_count, blue_persent
    right_count = 0
    left_count = 0
    straight = False  # 直行信号
    left = False  # 左移信号
    left2 = False  # 遇到右边缘且前方有障碍
    right = False  # 右移
    right2 = False  # 遇到左边缘且前方有障碍
    SSR.change_action_value("turn_left", 1)
    time.sleep(0.8)


    if org_img is None:
        print('No image')
        time.sleep(5)
    # frame = cv2.resize(OrgFrame, (r_w, r_h), interpolation=cv2.INTER_LINEAR)

    while True:
        # OrgFrame = copy.copy(org_img)
        # cv2.imshow('init', OrgFrame)
        # cv2.waitKey(3)3
        tangent_used = get_angle('blue')
        head_degree = 410 - forward_count * 3
        if head_degree < 380:
            head_degree = 380
        if move_count % 6 == 0:  # abled
            judge = True
            while judge:
                print("head up")
                time.sleep(0.5)
                SSR.serial_setServo(20, head_degree, 1000)
                time.sleep(1)
                tangent_used = get_angle('blue')
                #                 OrgFrame = org_img
                #                 #print(org_img)
                #                 #cv2.imshow('init', OrgFrame)
                #                 hsv = cv2.cvtColor(OrgFrame, cv2.COLOR_BGR2HSV)
                #                 hsv = cv2.GaussianBlur(hsv, (3, 3), 0)
                #                 Imask = cv2.inRange(hsv, color_dict['black_obstacle']['Lower'], color_dict['black_obstacle']['Upper'])
                #                 # Imask = cv2.erode(Imask, None, iterations=2)
                #                 Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
                #                 cv2.imshow('black', Imask)
                #
                #                 Imask2 = cv2.inRange(hsv, color_dict['blue']['Lower'], color_dict['blue']['Upper'])
                #                 Imask2 = cv2.erode(Imask2, None, iterations=2)
                #                 Imask2 = cv2.dilate(Imask2, np.ones((3, 3), np.uint8), iterations=2)
                #                 _, cnts2, hierarchy2 = cv2.findContours(Imask2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                #                 cv2.imshow('blue', Imask2)
                #
                #
                #                 areaMaxContour, area_max = getAreaMaxContour(cnts2)
                #                 image_line_detection = np.zeros(Imask2.shape, np.uint8)
                #
                #                 image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)
                #
                #                 lines = cv2.HoughLinesP(image_line_detection, 1, np.pi / 180, 30, 300, 5)
                #                 tangent_list = []
                #                 length_list = []
                #                 image_copy=copy.copy(org_img)
                if tangent_used is not None:
                    #                     for line in lines:
                    #                         x1, y1, x2, y2 = line[0]
                    #                         if y1 < 410 and x2 - x1 != 0:
                    #                             tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                    #                             if tangent_abs < 1:
                    #                                 length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    #                                 tangent_list.append([x1, y1, x2, y2])
                    #                     if len(tangent_list)>0:
                    #                         [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
                    #                         tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
                    #                     else:
                    #                         tangent_used=0
                    #                         [x1_f, y1_f, x2_f, y2_f]=[0,0,0,0]
                    #
                    #                     print('angle is',tangent_used)
                    #
                    #                     cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 1)

                    if 1.20 > tangent_used > 0.07:
                        print("should turn left little")
                        SSR.change_action_value("turn_left", 2)
                        time.sleep(0.5)
                    elif tangent_used > 1.20:
                        print("should turn left large")
                        SSR.change_action_value("turn_left", 2)
                        time.sleep(0.5)
                    elif -0.07 > tangent_used > -1.20:
                        print("should turn right little")
                        SSR.change_action_value("turn_right", 2)
                        time.sleep(0.5)
                    elif tangent_used < - 1.20:
                        print("should turn right large")
                        SSR.change_action_value("turn_right", 2)
                        time.sleep(0.5)
                    else:
                        print('judge done and go')
                        judge = False
                    # print(y1_f,y2_f,area_max)
                    # cv2.imshow("image_line1", image_copy)
                else:
                    judge = False
            print("head down and go")
            # print(step)
            SSR.serial_setServo(20, 310, 1000)
            time.sleep(1)
        # continue

#         OrgFrame = copy.copy(org_img)
        OrgFrame = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        # print(org_img)
        # cv2.imshow('init', OrgFrame)
        hsv = cv2.cvtColor(OrgFrame, cv2.COLOR_BGR2Lab)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 3)
        #Imask = cv2.inRange(hsv, color_dict['black_obstacle']['Lower'], color_dict['black_obstacle']['Upper'])
        Imask = cv2.inRange(hsv, lab_range['black'][0],lab_range['black'][1])
        # Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)
#         cv2.imshow('black', Imask)
#         cv2.waitKey(3)

        # cv2.imshow('blue', Imask2)
        obscle_area_blue = 0
        # 当遇到蓝色门槛时停止
        #             for c in cnts2:
        #                 obscle_area_blue += math.fabs(cv2.contourArea(c))
        #             if obscle_area_blue > 0.15 * 640 * 480:
        #                 cv2.destroyAllWindows()
        #                 SSR.change_action_value('1', 1)
        #
        #                 break
        # 划分roi区域，对每个区域检测边缘，得到最大边缘，如果存在则该区域为障碍
        n = -1
        obscle_list = []
        obscle_list_last = []
        for r in roi_mine:
            n += 1
            blobs = Imask[r[0]:r[1], r[2]:r[3]]
            _, cnts, hierarchy = cv2.findContours(blobs, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnt_large, _ = getAreaMaxContour(cnts, 200)
            #####################################change 80 to 200##########################
            if cnt_large is not None:
                # print(area_max)
                lefttop_x = r[2]
                lefttop_y = r[0]
                rightbottom_x = r[3]
                rightbottom_y = r[1]
                cv2.rectangle(OrgFrame, (lefttop_x, lefttop_y), (rightbottom_x, rightbottom_y), (0, 255, 255), 2)
                # cv2.imshow("what",org_img)
                obscle_list.append([n // 9, n % 9])
        s1 = [[5, 4], [5, 3], [5, 2], [5, 6], [5, 5], [4, 5], [4, 4], [4, 3], [3, 3], [3, 4], [3, 5]]
        s1_1 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7]]
        s1_2 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 8]]
        s1_3 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0]]
        s2_1 = [[5, 1], [5, 0], [5, 3], [5, 2]]
        s2_2 = [[5, 4], [5, 5], [5, 6]]
        s3_1 = [[5, 5], [5, 6], [5, 7], [5, 8]]
        s3_2 = [[5, 4], [5, 3], [5, 2]]
        s4_1 = [[5, 1], [5, 0], [5, 3], [5, 2], [5, 4], [5, 5], [5, 6]]
        s4_2 = [[5, 7], [5, 8]]
        s5_1 = [[5, 5], [5, 6], [5, 7], [5, 8], [5, 3], [5, 2], [5, 4]]
        s5_2 = [[5, 1], [5, 0]]
        s_edge_left = [[4, 0], [3, 0], [2, 0], [1, 0]]
        s_edge_right = [[4, 8], [3, 8], [2, 8], [1, 8]]
        s_adjust = [[4, 4], [4, 3], [4, 2], [4, 1], [4, 5], [4, 6], [4, 7], [4, 0], [4, 8]]
        s_under = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0], [5, 8], [4, 4], [4, 3], [4, 2],
                   [4, 1], [4, 5], [4, 6], [4, 7], [4, 0], [3, 8], [3, 4], [3, 3], [3, 2], [3, 1], [3, 5], [3, 6],
                   [3, 7], [3, 0], [3, 8]]
        # if all(s not in obscle_list for s in s1_1) and (all(s not in obscle_list for s in s1_2) or ([5,0] in obscle_list and all(s in obscle_list for s in s_edge_left)) or ([5,8] in obscle_list and all(s in obscle_list for s in s_edge_right))):

        if all(s not in obscle_list for s in s1):
            straight = True
            left = False
            left2 = False
            right = False
            right2 = False
        elif all(s in obscle_list for s in s_edge_left) and any(s in obscle_list for s in s1_2):
            straight = False
            left = False
            left2 = False
            right = False
            right2 = True
            if [5, 4] in obscle_list:
                k = 2
            elif [5, 5] in obscle_list or [5, 6] in obscle_list:
                k = 4
            elif [5, 7] in obscle_list or [5, 8] in obscle_list:
                k = 6
        elif all(s in obscle_list for s in s_edge_right) and any(s in obscle_list for s in s1_3):
            straight = False
            left = False
            left2 = True
            right = False
            right2 = False
            if [5, 4] in obscle_list:
                k = 2
            elif [5, 3] in obscle_list or [5, 2] in obscle_list:
                k = 4
            elif [5, 1] in obscle_list or [5, 0] in obscle_list:
                k = 6
        elif all(s not in obscle_list for s in s2_1) and all(s not in obscle_list_last for s in s2_1) and any(
                s in obscle_list for s in s2_2):
            straight = False
            left = True
            left2 = False
            right = False
            right2 = False
        elif all(s not in obscle_list for s in s3_1) and all(s not in obscle_list_last for s in s3_1) and any(
                s in obscle_list for s in s3_2):
            straight = False
            left = False
            left2 = False
            right = True
            right2 = False
        elif all(s not in obscle_list for s in s4_1) and all(s not in obscle_list_last for s in s2_1) and any(
                s in obscle_list for s in s4_2):
            straight = False
            left = True
            left2 = False
            right = False
            right2 = False
        elif all(s not in obscle_list for s in s5_1) and all(s not in obscle_list_last for s in s3_1) and any(
                s in obscle_list for s in s5_2):
            straight = False
            left = False
            left2 = False
            right = True
            right2 = False
        cv2.putText(OrgFrame,
                    "straight:" + str(straight) + " left:" + str(left) + " right:" + str(right) + " left2:" + str(
                        left2) + " right2:" + str(right2),
                    (10, OrgFrame.shape[0] - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        cv2.putText(OrgFrame, "step:" + str(step),
                    (10, OrgFrame.shape[0] - 55), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        # cv2.putText(OrgFrame,"finish:"+str(obscle_finish),(10,OrgFrame.shape[0]-75),cv2.FONT_HERSHEY_SIMPLEX,0.65,(0,0,255),2)

        # new_writer.write(OrgFrame)
        # cv2.imshow('frame', OrgFrame)
        # cv2.waitKey(3)
        if straight:
            time.sleep(0.2)
            SSR.change_action_value("go_middle_change_3", 1)  # 前进
            time.sleep(1)
            # SSR.change_action_value("turn_right", 1)
            # time.sleep(0.5)
            #SSR.change_action_value("hide_arm", 1)
            time.sleep(0.5)
            print("forward")
            move_count += 1
            forward_count += 2
            # move_count+=1
            # count += 1
        elif left:
            print('left')
            SSR.change_action_value("left_move_new_8", 1)  # 左移
            time.sleep(0.5)
            move_count += 1
        elif right:
            print('right')
            SSR.change_action_value("right_move_new_4", 1)  # 右移
            time.sleep(0.5)
            move_count += 1
            right_count += 1
            if right_count >= 2:
                time.sleep(0.5)
                print('right_count is:', right_count)
                SSR.change_action_value("turn_right", 1)
                time.sleep(0.5)
                right_count = 0
        elif left2 is True:
            print('left2')
            SSR.change_action_value("left_move_new_8", 1)  # 左移5次
            time.sleep(0.5)
            move_count += 1
        elif right2 is True:
            print('right2')
            SSR.change_action_value("right_move_new_4", 1)  # 右移5次
            time.sleep(0.5)
            move_count += 1
        else:
            print('atuo move')
            # SSR.change_action_value("1", 1)  # 静止
            SSR.change_action_value("go_middle_change_3", 1)  # 右移5次
            time.sleep(0.5)
#             SSR.change_action_value("hide_arm", 1)
            time.sleep(0.5)
            move_count += 1
            forward_count += 1
        print(OrgFrame.shape)
        print('move count is', move_count)
        print('forward_count is:', forward_count)
#         cv2.imshow('frame', OrgFrame)
#         cv2.waitKey(3)
        if blue_persent > 15:
            print('.....................blue....',blue_persent)
            break
    time.sleep(2)
    SSR.change_action_value("go_middle_stair", 4)  # 右移5次
    time.sleep(5)
    print('end of mine')
    time.sleep(2)


# 第4关：挡板
def flip():
    print('start fliping')
    SSR.change_action_value("flipp", 1)
    time.sleep(18)
    SSR.change_action_value("back_fast", 5)
    time.sleep(8)
    SSR.change_action_value("turn_left", 4)
    time.sleep(4)
    SSR.change_action_value("left_move_new_8", 2)
    time.sleep(2)


# 第5关：蓝色Π形门
def door_cali(rw, rh):
    global org_img, door_angle_cnt, door_double_cali, turn_head_door
    border = cv.copyMakeBorder(org_img, 12, 12, 16, 16, borderType=cv.BORDER_CONSTANT,
                               value=(255, 255, 255))  # 扩展白边，防止边界无法识别
    handling = cv.resize(border, (rw, rh), interpolation=cv.INTER_CUBIC)  # 将图片缩放
    roi = handling[5:110, :]
    frame_gauss = cv.GaussianBlur(roi, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv.cvtColor(frame_gauss, cv.COLOR_BGR2Lab)  # 将图片转换到HSV空间运算

    blue_door = cv.inRange(frame_hsv, lab_range['blue'][0],
                           lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    open_pic = cv.morphologyEx(blue_door, cv.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed_pic = cv.morphologyEx(open_pic, cv.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    mean_pic = cv.medianBlur(closed_pic, 11)
    cv.imshow('mean', mean_pic)
    #     cv.waitKey(27)
    contours, hierarchy = cv.findContours(closed_pic, cv.RETR_EXTERNAL,
                                          cv.CHAIN_APPROX_NONE)  # 找出轮廓
    sum_contour = getSumContour(contours, 80)
    x1, y1, w1, h1 = cv.boundingRect(sum_contour)  # 用最小的矩形圈住这个轮廓
    rect1 = cv.minAreaRect(sum_contour)  # 返回最小外接矩形的中心（x，y），（宽度，高度），旋转角度
    box = cv.boxPoints(rect1)
    cx = (box[0][0] + box[2][0]) / 2
    cy = (box[0][1] + box[2][1]) / 2
    print(cx)
    box = np.int0(box)
    cv.drawContours(roi, [box], 0, (0, 255, 0), 2)
    cv.circle(roi, radius=2, center=(int(cx), int(cy)), color=(0, 0, 255), thickness=5)
    cv.imshow('rect', roi)
    cv.waitKey(27)
    h1 = rect1[1][1]
    w1 = rect1[1][0]
    if cx > 160 and h1 < 90:
        SSR.change_action_value("right_move_new_4", 1)
        time.sleep(1.2)

    elif cx < 160 and h1 < 90:
        SSR.change_action_value("left_move_new_8", 1)
        time.sleep(1.2)

    elif cx < 155:
        SSR.change_action_value("left_move_new_8", 1)
        time.sleep(1.2)

    elif cx > 175:
        SSR.change_action_value("right_move_new_4", 1)
        time.sleep(1.2)

    else:
        for i in range(7):
            SSR.change_action_value("go_middle_change_3", 5)
            time.sleep(2.5)
    #         state = 'before_bridge'
    #         return state
    #     if turn_head_door:
    #         door_double_cali = 2
    #         Board.setPWMServoPulse(1, 1000, 500)  # 控制上下转动，参数2为位置值，范围0-1000，对应0-240度，500为时间，单位毫秒
    #         Board.setPWMServoPulse(2, 1500, 500)  # 控制左右转动，参数
    #         time.sleep(1.2)
    return


def door_angle():
    global org_img, door_angle_cnt, door_double_cali, turn_head_door
    SSR.serial_setServo(19, 450, 500)

    img_cali0 = org_img[200:480, 100:580]
    canny = cv2.Canny(img_cali0, 130, 200)
    lines = cv2.HoughLinesP(canny, 1, np.pi / 180, 90, minLineLength=50, maxLineGap=30)
    count_R = 0
    count_L = 0

    if lines is not None:
        sum = 0
        count = 0
        for line1 in lines:
            x1, y1, x2, y2 = line1[0]
            k = np.double((y1 - y2) / (x1 - x2))
            print(k)
            if 1 > k > 0.04:
                count_L += 1
            if -1 < k < -0.04:
                count_R += 1
            cv2.line(img_cali0, (x1, y1), (x2, y2), (0, 0, 255), 2)
    if count_L <= 2 and count_R <= 2:
        print('No motion')
    elif count_L > count_R:
        print('left move')
        SSR.change_action_value("turn_left", 1)
        time.sleep(1.2)
    elif count_L < count_R:
        print('right move')
        SSR.change_action_value("turn_right", 1)
        time.sleep(1.2)


def find_center_of_next():
    global org_img, door_cnt, door_end, r_w, r_h
    # r_w = 640
    # r_h = 480
    SSR.change_action_value("1", 1)
    time.sleep(0.5)
    SSR.serial_setServo(20, 450, 500)
    time.sleep(0.5)
    SSR.serial_setServo(19, 500, 500)
    time.sleep(0.5)
    while org_img is None:
        time.sleep(0.5)
        print('delay 0.5s')

    org_img_copy = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_green = cv2.inRange(frame_hsv, lab_range['white'][0], lab_range['white'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    #cv2.imshow('closed', closed)

    (image, contours, hierarchy) = cv2.findContours(closed[200:480, :], cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像
    areaMaxContour, area_max = getAreaMaxContour(contours, 50)

    center = (1, 1)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)
        center, w_h, angle = rect  # 中心点 宽高 旋转角度
        box = np.int0(cv2.boxPoints(rect))  # 点的坐标
    #         print('center is ',center)
    #         if center[0]<265:
    #             print('left move')
    #             SSR.change_action_value("left_move_new_8", 1)
    #             time.sleep(0.5)
    #         if center[0]>375:
    #             print('right move')
    #             SSR.change_action_value("right_move_new_4", 1)
    #             time.sleep(0.5)

    #     cv2.imshow('frame with center',org_img_copy)
    # rect = cv2.minAreaRect()  # 最小外接矩形
    # center, w_h, angle = rect  # 中心点 宽高 旋转角度
    # box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点

    image_line_detection = np.zeros(closed.shape, np.uint8)

    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    lines = cv2.HoughLinesP(image_line_detection[0:260], 1, np.pi / 180, 30, 300, 5)

    ################################调整转向部分##############################
    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y1 < 250 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    tangent_list.append([x1, y1, x2, y2])
        if len(tangent_list) > 0:
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print('angle is: ', tangent_used)

        #cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 1)
        #cv2.line(image_line_detection, (x1_f, y1_f), (x2_f, y2_f), 255, 4)
        #cv2.circle(image_copy, (int(center[0]), int(center[1])), 5, (255, 0, 0), 2)
        #cv2.imshow('contour', image_line_detection)
        #cv2.imshow("image_line1", image_copy)
        if 0.20 > tangent_used > 0.07:
            print("should turn left little")
            time.sleep(0.4)
            SSR.change_action_value("turn_left", 1)
            time.sleep(1.2)
        elif tangent_used > 0.20:
            print("should turn left large")
            time.sleep(0.4)
            SSR.change_action_value("turn_left", 1)
            time.sleep(1.2)
        elif -0.07 > tangent_used > -0.20:
            print("should turn right little")
            time.sleep(0.4)
            SSR.change_action_value("turn_right", 1)
            time.sleep(1.2)
        elif tangent_used < - 0.20:
            print("should turn right large")
            time.sleep(0.4)
            SSR.change_action_value("turn_right", 1)
            time.sleep(1)
        # print(y1_f, y2_f, area_max)
        else:
            #             SSR.serial_setServo(20, 550, 500)
            frame_door = cv2.inRange(frame_hsv[20:120, :], lab_range['blue'][0],
                                     lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            opened_door = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones(( ), np.uint8))  # 开运算 去噪点
            closed_door = cv2.morphologyEx(opened_door, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

            (image_door, contours_door, hierarchy_door) = cv2.findContours(closed_door, cv2.RETR_LIST,
                                                                           cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
            print(door_cnt)
            if (contours_door is None) and door_cnt >= 0:
                print('go go go')
                time.sleep(2)
                #SSR.change_action_value("go_middle_change_3", 10)
                SSR.change_action_value("go_middle_stair", 10)
                time.sleep(7)
                door_end = 1

            elif contours_door is not None:

                if len(contours_door) < 2 and door_cnt == 0:  # changed
                    SSR.change_action_value("left_move_new_8", 2)
                    time.sleep(2)
                if len(contours_door) < 2 and door_cnt > 0:
                    print('one stick')
                    print('go go go -----')
                    time.sleep(4)
                    SSR.change_action_value("go_middle_change_3", 10)
                    time.sleep(10)
                    print('go done')
                    door_end = 1
                    return

                areaMaxContour_door_1, area_max_door = getAreaMaxContour(contours_door, 100)
                print('two', len(contours_door), area_max_door)
                if (areaMaxContour_door_1 is not None) and len(contours_door) >= 2 and door_end != 1:
                    # print(len(contours_door))
                    # print(len(areaMaxContour_door_1))
                    # print('it is ', contours_door)
                    # contours_door.remove(areaMaxContour_door_1)
                    #                     print('it is ', contours_door.index(areaMaxContour_door_1))
                    #                     contours_door.pop(contours_door.index(areaMaxContour_door_1))
                    print('calsecond')
                    areaMaxContour_door_2, area_max_door = getAreaSecondMaxContour(contours_door, area_max_door)
                    if areaMaxContour_door_2 is not None:
                        print('second not None')
                        areaMaxContour_door = [areaMaxContour_door_1, areaMaxContour_door_2]
                        door_line_detection = np.zeros(closed_door.shape, np.uint8)
                        door_line_detection = cv2.drawContours(door_line_detection, areaMaxContour_door, -1, 255, 1)
                        #                         print(areaMaxContour_door_1)
                        #                         for i in range(r_w):
                        #                             if frame_door[i][]
                        arr1_min = (np.array(door_line_detection[40, :]) != 0).argmax(axis=0)
                        arr1_max = r_w - (np.flipud(np.array(door_line_detection[40, :])) != 0).argmax(axis=0)
                        arr2_min = (np.array(door_line_detection[80, :]) != 0).argmax(axis=0)
                        arr2_max = r_w - (np.flipud(np.array(door_line_detection[80, :])) != 0).argmax(axis=0)
                        arr3_min = (np.array(door_line_detection[60, :]) != 0).argmax(axis=0)
                        arr3_max = r_w - (np.flipud(np.array(door_line_detection[60, :])) != 0).argmax(axis=0)
                        #                         print('min and max',arr1_min,arr1_max)
                        #                         print(np.array(door_line_detection[70,:]))
                        mid = (arr1_min + arr1_max + arr2_min + arr2_max + arr3_min + arr3_max) / 6
                        print("mid = ", mid)

                        if mid < 295:
                            print('left move')
                            SSR.change_action_value("left_move_new_8", 1)
                            time.sleep(0.8)
                        elif mid > 345:
                            print('right move')
                            SSR.change_action_value("right_move_new_4", 1)
                            time.sleep(0.8)
                        else:
                            print("======================= already in the middle ==========================")
                            door_cnt += 1
                            time.sleep(1)
#                             SSR.change_action_value("go_middle_change_3", 9)
                            SSR.change_action_value("go_middle_stair", 10)
                            time.sleep(7)
                            door_end = 1
                            return
                        #cv2.imshow("door line", door_line_detection)
                    elif (areaMaxContour_door_2 is None) and door_cnt >= 1:
                        print('one stick')
                        print('go go go -----')
                        time.sleep(4)
#                         SSR.change_action_value("go_middle_change_3", 8)
                        SSR.change_action_value("go_middle_stair", 5)
                        time.sleep(6)
                        SSR.change_action_value("turn_left", 2)
                        time.sleep(1.2)
                        SSR.change_action_value("go_middle_stair", 5)
#                         SSR.change_action_value("go_middle_change_3", 8)
                        time.sleep(6)
                        print('go done')
                        door_end = 1
                        return
                    else:
                        #                         door_line_detection = np.zeros(closed_door.shape, np.uint8)
                        #                         door_line_detection = cv2.drawContours(door_line_detection, areaMaxContour_door_1, -1, 255, 1)
                        #                         arr1_min=(np.array(door_line_detection[40,:])!=0).argmax(axis=0)
                        #                         arr1_max=r_w-(np.flipud(np.array(door_line_detection[40,:]))!=0).argmax(axis=0)
                        #                         arr2_min=(np.array(door_line_detection[50,:])!=0).argmax(axis=0)
                        #                         arr2_max=r_w-(np.flipud(np.array(door_line_detection[50,:]))!=0).argmax(axis=0)
                        #                         arr3_min=(np.array(door_line_detection[30,:])!=0).argmax(axis=0)
                        #                         arr3_max=r_w-(np.flipud(np.array(door_line_detection[30,:]))!=0).argmax(axis=0)
                        # #                         print('min and max',arr1_min,arr1_max)
                        # #                         print(np.array(door_line_detection[70,:]))
                        #                         mid = (arr1_min + arr1_max + arr2_min + arr2_max + arr3_min + arr3_max) / 6
                        #                         print("mid = ", mid)
                        #                         if 160 <mid < 440:
                        #                             print('left move')
                        #                             SSR.change_action_value("left_move_new_8", 1)
                        #                             time.sleep(0.5)
                        #                         elif mid < 160:
                        #                             print('right move')
                        #                             SSR.change_action_value("right_move_new_4", 2)
                        #                             time.sleep(1)
                        # #                         elif mid < 310:
                        # #                             print('right move')
                        # #                             SSR.change_action_value("right_move_new_4", 1)
                        # #                             time.sleep(0.5)
                        #                         else:
                        #                             print("======================= already in the middle ==========================")
                        #                             door_cnt += 1
                        #                             SSR.change_action_value("go_middle_change_3", 4)
                        #                             time.sleep(2)
                        check_stick()
                elif door_cnt >= 0 and door_end != 1:
                    print('no stick')
                    print('go go go -----')
                    time.sleep(4)
#                     SSR.change_action_value("go_middle_change_3", 7)
                    SSR.change_action_value("go_middle_stair", 5)
                    time.sleep(6)
                    SSR.change_action_value("turn_left", 1)
                    time.sleep(1.2)
                    SSR.change_action_value("go_middle_stair", 5)
#                     SSR.change_action_value("go_middle_change_3", 8)
                    time.sleep(6)
                    print('go done')
                    door_end = 1
                    return

            cv2.imshow("closed_door", closed_door)
    else:
        print('no line')
        SSR.change_action_value("go_middle_stair", 2)
        time.sleep(2)


def door():
    while True:
        find_center_of_next()
        #     door_cali(320, 240)
        if door_end == 1:
            print('door end')
            time.sleep(2)
            find_area_percent()
            break


#         cv2.waitKey(3)
def find_area_percent():
    percent = 0
    straight_count=0
    SSR.serial_setServo(20, 350, 500)
    time.sleep(1)
    while True:
        org_img_copy = cv2.resize(org_img.copy(), (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 3)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
        frame_green = cv2.inRange(frame_hsv, lab_range['white'][0], lab_range['white'][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
        (image, contours, hierarchy) = cv2.findContours(closed[200:480, :], cv2.RETR_LIST,
                                                        cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
        areaMaxContour, area_max = getAreaMaxContour(contours, 50)
        if areaMaxContour is not None:
            cv2.drawContours(org_img_copy, [areaMaxContour], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
        percent = area_max / (r_w * r_h) * 100
        print(percent, area_max)
        tangent_used = get_angle('white')
        # cv2.imshow('im', closed)
        # cv2.imshow('im1', org_img_copy)
        # cv2.waitKey(3)
        if percent < 1:
            print('going out of mine',percent)
            cv2.imshow('im', closed)
            cv2.waitKey(3)
            break
        elif tangent_used is not None:
            if 0.20 > tangent_used > 0.07:
                print("should turn left little")
                time.sleep(0.4)
                SSR.change_action_value("turn_left", 1)
                time.sleep(1.2)
            elif tangent_used > 0.20:
                print("should turn left large")
                time.sleep(0.4)
                SSR.change_action_value("turn_left", 1)
                time.sleep(1.2)
            elif -0.07 > tangent_used > -0.20:
                print("should turn right little")
                time.sleep(0.4)
                SSR.change_action_value("turn_right", 1)
                time.sleep(1.2)
            elif tangent_used < - 0.20:
                print("should turn right large")
                time.sleep(0.4)
                SSR.change_action_value("turn_right", 1)
                time.sleep(1)
            else:
                SSR.change_action_value("go_middle_stair", 3)
                time.sleep(1.5)
        else:
            SSR.change_action_value("go_middle_stair", 3)
            time.sleep(1.5)
            straight_count=straight_count+1
            if straight_count>=3:
                break

# 第6关：独木桥-- 2
def bridge(r_w, r_h):
    global org_img, step, reset, skip, debug, bridge_end

    if cap.isOpened():
        print(org_img.shape)
    print('进入bridge')
    step = 0
    cnt = 0
    SSR.serial_setServo(19, 500, 500)  # NO3机器人，初始云台角度
    SSR.serial_setServo(20, 350, 500)
    time.sleep(0.5)
    # global state, state_sel, org_img, step, reset, skip, debug
    # org_img = cv2.imread("../data/video1/2611.jpg")
    # org_img[150:160,260:270,:] = [0,0,0]#230,330...480,510
    # cv2.imshow("a", org_img)

    '''
    print(org_img[300,410,:])
    part = org_img[160:460,270:430,:]
    part = cv2.cvtColor(part, cv2.COLOR_BGR2HSV)
    print("B max:", part[:,:,0].max())
    print("B min:", part[:,:,0].min())
    print("G max:", part[:,:,1].max())
    print("G min:", part[:,:,1].min())
    print("R max:", part[:,:,2].max())
    print("R min:", part[:,:,2].min())
    '''
    '''
    if (state == 2 or state == 6 or state == 8) and state_sel == 'bridge':  # 初始化
        print('进入bridge')
        step = 0
        cnt = 0
        PWMServo.setServo(1, 2000, 500)  # NO3机器人，初始云台角度
        PWMServo.setServo(2, 1510, 500)
        time.sleep(0.5)
        #SSR.running_action_group('00', 1)
        # time.sleep(0.5) #等待摄像头稳定
    elif (state == 2 or state == 6 or state == 8) and state_sel is None:  # 跳关情况
        state_sel = state_choose(320, 240)
        return
    else:
        return
    '''

    # while (state == 2 or state == 6 or state == 8) and state_sel == 'bridge':  # 初始化
    while (True):
        '''
        if reset == 1:  # 是否为重置情况
            # PWMServo.setServo(1, 2100, 500) # NO2机器人，初始云台角度
            # PWMServo.setServo(2, 1510, 500)
            PWMServo.setServo(1, 2000, 500)  # NO3机器人，初始云台角度
            PWMServo.setServo(2, 1510, 500)
            reset = 0
            step = 0
            cnt = 0
            cv2.destroyAllWindows()
            SSR.running_action_group('0', 1)

        if skip == 1:  # 跳至下一关
            cv2.destroyAllWindows()
            state += 1
            skip = 0
            state_sel = None
            SSR.running_action_group('666', 1)
            time.sleep(5)
            break
        '''
        t1 = cv2.getTickCount()
        border = cv2.copyMakeBorder(org_img[:,0:520], 12, 12, 16, 16, borderType=cv2.BORDER_CONSTANT,
                                    value=(255, 255, 255))  # 扩展白边，防止边界无法识别
        org_img_copy = cv2.resize(border, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
        frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
        color = judge_color()
        frame_green = cv2.inRange(frame_hsv, lab_range[color][0],
                                  lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
        opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
        # print(closed)

        (_, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE

        areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        percent = round(area_max * 100 / (r_w * r_h), 2)
        # if debug:
        if True:
            cv2.imshow('closed', closed)  # 显示图像
            cv2.drawContours(org_img_copy, contours, -1, (255, 0, 255), 1)
            cv2.putText(org_img_copy, 'area:' + str(percent) + '%', (100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0),
                        2)  # (0, 0, 255)BGR
        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)
            # center, w_h, angle = rect  # 中心点 宽高 旋转角度
            box = np.int0(cv2.boxPoints(rect))  # 点的坐标

            top_left = areaMaxContour[0][0]
            top_right = areaMaxContour[0][0]
            bottom_left = areaMaxContour[0][0]
            bottom_right = areaMaxContour[0][0]
            for c in areaMaxContour:  # 遍历找到四个顶点
                if c[0][0] + 1.5 * c[0][1] < top_left[0] + 1.5 * top_left[1]:
                    top_left = c[0]
                if (r_w - c[0][0]) + 1.5 * c[0][1] < (r_w - top_right[0]) + 1.5 * top_right[1]:
                    top_right = c[0]
                if c[0][0] + 1.5 * (r_h - c[0][1]) < bottom_left[0] + 1.5 * (r_h - bottom_left[1]):
                    bottom_left = c[0]
                if c[0][0] + 1.5 * c[0][1] > bottom_right[0] + 1.5 * bottom_right[1]:
                    bottom_right = c[0]
            # angle_top = - math.atan((top_right[1] - top_left[1]) / (top_right[0] - top_left[0])) * 180.0 / math.pi
            # angle_bottom = - math.atan((bottom _right[1] - bottom_left[1]) / (bottom _right[0] - bottom_left[0])) * 180.0 / math.pi
            top_center_x = int((top_right[0] + top_left[0]) / 2)
            top_center_y = int((top_right[1] + top_left[1]) / 2)
            bottom_center_x = int((bottom_right[0] + bottom_left[0]) / 2)
            bottom_center_y = int((bottom_right[1] + bottom_left[1]) / 2)
            center_x = int((top_center_x + bottom_center_x) / 2)
            # if debug:
            if True:
                cv2.drawContours(org_img_copy, [box], 0, (0, 0, 255), 2)  # 将大矩形画在图上
                cv2.circle(org_img_copy, (top_right[0], top_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_left[0], top_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_right[0], bottom_right[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_left[0], bottom_left[1]), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (top_center_x, top_center_y), 5, [0, 255, 255], 2)
                cv2.circle(org_img_copy, (bottom_center_x, bottom_center_y), 5, [0, 255, 255], 2)
                cv2.line(org_img_copy, (top_center_x, top_center_y), (bottom_center_x, bottom_center_y), [0, 255, 255],
                         2)  # 画出上下中点连线
            if math.fabs(top_center_x - bottom_center_x) <= 1:  # 得到连线的角度
                angle = 90
            else:
                angle = - math.atan(
                    (top_center_y - bottom_center_y) / (top_center_x - bottom_center_x)) * 180.0 / math.pi
        else:
            angle = 90
            center_x = 0.5 * r_w
        # if debug:
        if True:
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            # print('fps: %d' %(fps))
            # cv2.putText(org_img_copy, "step:" + str(step), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "angle:" + str(int(angle)), (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0),
                        2)  # (0, 0, 255)BGR
            cv2.putText(org_img_copy, "center_x:" + str(int(center_x)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 0, 0), 2)  # (0, 0, 255)BGR
            # cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(1)

        if step == 0:  # 接近独木桥阶段
            SSR.change_action_value("go_middle_change_3", 1)  # 前进
            time.sleep(1)
            print("step = 0, go forward")
            if percent > 17:  # 绿色区域大于30%
                print('percent > 20')
#                 SSR.change_action_value('1', 1)  # 立正
                step = 1
        elif step == 1:  # 独木桥阶段
            if percent < 5:  # 接近终点，直行2步离开
                time.sleep(1)
                SSR.change_action_value("go_middle_change_3", 5)  # 前进
                time.sleep(2)
                print('End point:go forward')
#                 SSR.change_action_value("1", 1)  # 前进
                # state_sel = None
                # state += 1
                step = -1  # 出于踢球双线程的考虑
                cv2.destroyAllWindows()
                bridge_end = 1
                break
            if 0 < angle < 80:  # 右转
                SSR.change_action_value('turn_right', 1)
                time.sleep(0.6)
                SSR.change_action_value('go_middle_change_3', 2)
                time.sleep(1.2)
                print('angle:turn right')
            elif -80 < angle < 0:  # 左转
                SSR.change_action_value('turn_left', 1)
                time.sleep(0.6)
                SSR.change_action_value('go_middle_change_3', 2)
                time.sleep(1.2)
                print('angle:turn left')
            elif angle <= -80 or angle >= 80:
                if center_x > 0.6 * r_w:  # 右移
                    SSR.change_action_value('right_move_new_4', 1)
                    time.sleep(0.8)
                    SSR.change_action_value('go_middle_change_3', 2)
                    time.sleep(1.2)
                    print('aim_point:turn right')
                elif center_x < 0.4 * r_w:  # 左移
                    SSR.change_action_value('left_move_new_8', 1)
                    time.sleep(0.6)
                    SSR.change_action_value('go_middle_change_3', 2)
                    time.sleep(1.2)
                    print('aim_point:turn left')
                    # time.sleep(0.1)
                elif 0.4 * r_w <= center_x <= 0.6 * r_w:  # 走三步
                    time.sleep(0.5)
                    SSR.change_action_value('go_middle_change_3', 2)
                    time.sleep(1.2)
                    print('we go faster straight!!!!3 step')
#                     SSR.change_action_value('1', 1)
                    cnt += 1
                    if cnt == 3:  # 走四次后更低头
                        SSR.serial_setServo(19, 500, 500)
                        SSR.serial_setServo(20, 300, 500)
                        time.sleep(0.6)


# 第7关：踢球
def find_center():
    global org_img
    r_w = 640
    r_h = 480
    while org_img is None:
        time.sleep(0.5)
        print('delay 0.5s')

    org_img_copy = cv2.resize(org_img, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    color = judge_color()
    frame_green = cv2.inRange(frame_hsv, lab_range[color][0], lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    cv2.imshow('closed', closed)

    (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像

    areaMaxContour, area_max = getAreaMaxContour(contours)
    center = (1, 1)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)
        center, w_h, angle = rect  # 中心点 宽高 旋转角度
        box = np.int0(cv2.boxPoints(rect))  # 点的坐标
        print('center is ', center)
        if center[0] < 270:
            print('left move')
            SSR.change_action_value("left_move_new_8", 1)
            time.sleep(1.2)
        if center[0] > 370:
            print('right move')
            SSR.change_action_value("right_move_new_4", 1)
            time.sleep(1.2)
    cv2.circle(org_img_copy, (int(center[0]), int(center[1])), 5, (255, 0, 0), 2)
    cv2.imshow('frame with center', org_img_copy)
    # rect = cv2.minAreaRect()  # 最小外接矩形
    # center, w_h, angle = rect  # 中心点 宽高 旋转角度
    # box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点

    image_line_detection = np.zeros(closed.shape, np.uint8)

    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    lines = cv2.HoughLinesP(image_line_detection, 1, np.pi / 180, 30, 300, 5)

    ################################调整转向部分##############################
    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y1 < 410 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    tangent_list.append([x1, y1, x2, y2])
        if len(tangent_list) > 0:
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print('angle is: ', tangent_used)

        cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 1)
        if 0.20 > tangent_used > 0.07:
            print("should turn left little")
            SSR.change_action_value("turn_left", 1)
            time.sleep(0.5)
        elif tangent_used > 0.25:
            print("should turn left large")
            SSR.change_action_value("turn_left", 1)
            time.sleep(0.5)
        elif -0.07 > tangent_used > -0.25:
            print("should turn right little")
            SSR.change_action_value("turn_right", 1)
            time.sleep(0.5)
        elif tangent_used < - 0.25:
            print("should turn right large")
            SSR.change_action_value("turn_right", 1)
            time.sleep(0.5)
        # print(y1_f, y2_f, area_max)
        cv2.imshow("image_line1", image_copy)


def walk_long():
    # 定死的走路
    SSR.serial_setServo(20, 400, 500)
    print("forward")
    SSR.change_action_value("1", 1)  # 前进
    time.sleep(1)
    SSR.change_action_value("go_middle_stair", 5)  # 前进
    time.sleep(5)
    SSR.change_action_value("turn_left",2)
    time.sleep(3)
    SSR.change_action_value("go_middle_stair", 5)  # 前进
    time.sleep(5)
    SSR.change_action_value("turn_left",2)
    time.sleep(3)

    #SSR.change_action_value("left_move_new_8", 2)
    #time.sleep(3)
    #SSR.change_action_value("turn_left", 3)
    time.sleep(2)
    SSR.change_action_value("go_middle_stair", 5)  # 前进
    time.sleep(5)
    print("turn left little")
    SSR.change_action_value("turn_left", 9)
    time.sleep(16)

    print("forward")
    SSR.change_action_value("go_middle_stair", 10)  # 前进
    time.sleep(6)

    print("turn left little")
    SSR.change_action_value("turn_left", 5)
    time.sleep(3)

    print("forward")
    SSR.change_action_value("go_middle_stair", 5)  # 前进
    time.sleep(5)

    time_start_adjust = time.time()  # 第一次调整
    while True:
        find_center()
        cv2.waitKey(3)
        time_end_adjust = time.time()
        if time_end_adjust - time_start_adjust > 12:
            break
    print("forward")
    time.sleep(1)
    SSR.change_action_value("go_middle_change_3", 5)  # 前进#走进绿色预备区
    time.sleep(4)
    time_start_adjust = time.time()  # 第二次调整
    while True:
        find_center()
        cv2.waitKey(3)
        time_end_adjust = time.time()
        if time_end_adjust - time_start_adjust > 8:
            break
    time.sleep(2)
    SSR.change_action_value("go_middle_stair", 5)  # 前进#走到台阶前
    time.sleep(8)
    print('done')


def walk_short():
    # 定死的走路
    SSR.serial_setServo(20, 400, 500)
    print("forward")
    SSR.change_action_value("1", 1)  # 前进
    time.sleep(1)
    #     SSR.change_action_value("go_middle_stair", 12)  # 前进
    #     time.sleep(10)
    #     SSR.change_action_value("left_move_new_8", 2)
    #     time.sleep(3)
    #     SSR.change_action_value("turn_left", 3)
    #     time.sleep(4)
    SSR.change_action_value("go_middle_stair", 8)  # 前进
    time.sleep(6)
    print("turn left little")
    SSR.change_action_value("turn_left", 10)
    time.sleep(16)

    print("forward")
    SSR.change_action_value("go_middle_stair", 12)  # 前进
    time.sleep(6)

    time_start_adjust = time.time()
    while True:
        find_center()
        cv2.waitKey(3)
        time_end_adjust = time.time()
        if time_end_adjust - time_start_adjust > 15:
            break
    print("forward")
    SSR.change_action_value("go_middle_stair", 2)
    time.sleep(1)

    #     print("turn left little")
    #     SSR.change_action_value("turn_left", 5)
    #     time.sleep(3)
    #
    #     print("forward")
    #     SSR.change_action_value("go_middle_stair", 5)  # 前进
    #     time.sleep(5)

    #     time_start_adjust = time.time()  # 第一次调整
    #     while True:
    #         find_center()
    #         cv2.waitKey(3)
    #         time_end_adjust = time.time()
    #         if time_end_adjust - time_start_adjust > 15:
    #             break
    #     print("forward")
    #     time.sleep(1)
    #     SSR.change_action_value("go_middle_change_3", 10)  # 前进#走进绿色预备区
    #     time.sleep(4)
    #     time_start_adjust = time.time()  # 第二次调整
    #     while True:
    #         find_center()
    #         cv2.waitKey(3)
    #         time_end_adjust = time.time()
    #         if time_end_adjust - time_start_adjust > 15:
    #             break
    #     time.sleep(2)
    #     SSR.change_action_value("go_middle_stair", 10)  # 前进#走到台阶前
    #     time.sleep(8)
    print('done')


# 第8关：楼梯
# 第9关：下坡
def stairs():
    print("stair ")
    time_start_adjust = time.time()
    while True:
        find_center()
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

    SSR.change_action_value("1", 1)
    time.sleep(1)

    SSR.change_action_value("star_single_final", 1)
    time.sleep(12)

    #SSR.change_action_value("go_middle_stair", 2)
    #time.sleep(2)

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

    print('done')


# 第9关：终点
def end_door(width, height):
    global org_img, debug, isstop

    SSR.serial_setServo(19, 500, 500)
    time.sleep(0.8)
    SSR.serial_setServo(20, 400, 500)
    time.sleep(0.8)
    isstop = False
    door_count = 0
    no_door_count = 0

    print('进入end_door')
    # walk whatever
    SSR.change_action_value("go_middle_stair", 3)  # gogogo
    time.sleep(5)

    while True:
        print('loop')
        t1 = cv2.getTickCount()
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
        _, contours, hierarchy = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        areaMaxContour, area_max = getAreaMaxContour(contours, 25)  # 找出最大轮廓
        percent = round(100 * area_max / (width * height), 2)  # 最大轮廓的百分比

        if areaMaxContour is not None:
            rect = cv2.minAreaRect(areaMaxContour)  # 矩形框选
            box = np.int32(cv2.boxPoints(rect))  # 点的坐标
            if debug:
                cv2.drawContours(org_img_copy, [box], 0, (153, 200, 0), 2)  # 将最小外接矩形画在图上
        if debug:
            #             cv2.imshow('orginal frame', org_img)
            cv2.imshow('closed', closed)  # 显示图像
            cv2.putText(org_img_copy, 'area: ' + str(percent) + '%', (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)
            t2 = cv2.getTickCount()
            time_r = (t2 - t1) / cv2.getTickFrequency()
            fps = 1.0 / time_r
            cv2.putText(org_img_copy, "fps:" + str(int(fps)), (30, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
            #             cv2.moveWindow('orgFrame', img_center_x, 100)  # 显示框位置
            cv2.imshow('org_img_copy', org_img_copy)  # 显示图像
            cv2.waitKey(3)

        # 根据比例得到是否前进的信息
        if percent > 10:
            print(percent)
            print('stop')
            isstop = True
            # SSR.change_action_value("1",0)  # 静止
            time.sleep(0.1)
            door_count += 1
            no_door_count = 0
        elif door_count >= 10:
            no_door_count += 1
            if no_door_count >= 20:
                isstop = False
                print('gogogo')
                print(percent)
                SSR.change_action_value("go_middle_change_3", 8)  # gogogo
                time.sleep(6)
                break
        else:
            pass
        print("door_count = ", door_count, " no_door_count = ", no_door_count)


# others
def get_angle_red():
    global org_img, r_w, r_h
    org_img_copy = copy.copy(org_img)
    org_img_copy = cv2.resize(org_img_copy, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_lab = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)
    frame_red = cv2.inRange(frame_lab, lab_range['red'][0], lab_range['red'][1])
    opened = cv2.morphologyEx(frame_red, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    cv2.imshow('closed', closed)

    (image, contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
                                                    cv2.CHAIN_APPROX_NONE)  # 找出轮廓cv2.CHAIN_APPROX_NONE
    image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像

    areaMaxContour, area_max = getAreaMaxContour(contours)
    center = (1, 1)
    if areaMaxContour is not None:
        rect = cv2.minAreaRect(areaMaxContour)
        center, w_h, angle = rect  # 中心点 宽高 旋转角度
        box = np.int0(cv2.boxPoints(rect))  # 点的坐标
    image_line_detection = np.zeros(closed.shape, np.uint8)

    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    lines = cv2.HoughLinesP(image_line_detection, 1, np.pi / 180, 30, 300, 5)

    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if y1 < 410 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))
                    tangent_list.append([x1, y1, x2, y2])
        if len(tangent_list) > 0:
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  #####标准tangent
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print('angle is: ', tangent_used)
        percent = round(area_max * 100 / (r_w * r_h), 2)
        cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 3)
        print('persent: ', percent)
        cv2.circle(image_copy, (int(center[0]), int(center[1])), 5, (255, 0, 0), 2)
        cv2.imshow("image_line", image_copy)
        return tangent_used


def begin_red():
    while True:
        tangent_used = get_angle_red()
        if tangent_used is None:
            print('None')
            break
        elif 0.20 > tangent_used > 0.07:
            print("should turn left little")
            time.sleep(0.4)
            SSR.change_action_value("turn_left", 1)
            time.sleep(1.2)
        elif tangent_used > 0.20:
            print("should turn left large")
            time.sleep(0.4)
            SSR.change_action_value("turn_left", 1)
            time.sleep(1.2)
        elif -0.07 > tangent_used > -0.20:
            print("should turn right little")
            time.sleep(0.4)
            SSR.change_action_value("turn_right", 1)
            time.sleep(1.2)
        elif tangent_used < - 0.20:
            print("should turn right large")
            time.sleep(0.4)
            SSR.change_action_value("turn_right", 1)
            time.sleep(1)
        else:
            print('judge done')
            break


# 之后 after_bridge

########################## 主循环 ##########################

while True:
    if org_img is not None and ret:
        if Running:
            time.sleep(120)
            start_door(320,240)
            bridge(320, 240)
            obscle()
            
            flip()
            cv2.destroyAllWindows()
            door()
            cv2.destroyAllWindows()
            
            
            cross_trap_all()
            begin_red()
            walk_long()
            stairs()
            end_door(320, 240)
            Running = False
        else:
            print('Running is false')
            time.sleep(0.01)
            break
    else:
        print('image is empty')
        time.sleep(0.01)
        # cv2.destroyAllWindows()


