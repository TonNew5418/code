import math
import cv2
from running_parameters import r_w, r_h, lab_range
import numpy as np
import RobotControl.Serial_Servo_Running as SSR
import time
import copy

# 得到最大轮廓和对应的最大面积
def getAreaMaxContour(contours, area_min=4000):
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
    contour_area_max = 0
    area_max_contour = None
    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if max_area == contour_area_temp:
            continue
        elif contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 1500:  # 只有在面积大于4000时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓


# 得到全部的总的轮廓
def getSumContour(contours, area=1):
    contours_sum = None
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = c
            break
    for c in contours:
        area_temp = math.fabs(cv2.contourArea(c))
        if area_temp > area:
            contours_sum = np.concatenate((contours_sum, c), axis=0)  # 将所有轮廓点拼接到一起
    return contours_sum


# 判断当前地面的颜色
def judge_color(org_img):
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
    (contours_blue, hierarchy) = cv2.findContours(closed_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    (contours_green, hierarchy) = cv2.findContours(closed_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    areaMaxContour, area_max_blue = getAreaMaxContour(contours_blue)
    areaMaxContour, area_max_green = getAreaMaxContour(contours_green)
    if area_max_blue > area_max_green:
        color = 'blue'
    else:
        color = 'green'
    return color

def find_center(org_img):

    r_w = 640
    r_h = 480
    while org_img is None:
        time.sleep(0.5)
        print('delay 0.5s')

    org_img_copy = cv2.resize(org_img, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    color = judge_color(org_img)
    frame_green = cv2.inRange(frame_hsv, lab_range[color][0], lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    cv2.imshow('closed', closed)

    (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_LIST,
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
