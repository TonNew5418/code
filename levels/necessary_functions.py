import math
import cv2
from running_parameters import r_w, r_h, lab_range
import numpy as np


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
    (image, contours_blue, hierarchy) = cv2.findContours(closed_blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    (image, contours_green, hierarchy) = cv2.findContours(closed_green, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    areaMaxContour, area_max_blue = getAreaMaxContour(contours_blue)
    areaMaxContour, area_max_green = getAreaMaxContour(contours_green)
    if area_max_blue > area_max_green:
        color = 'blue'
    else:
        color = 'green'
    return color
