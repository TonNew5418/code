import cv2
import copy
import numpy as np
import time
from levels.necessary_functions import getAreaMaxContour, judge_color
import running_parameters as rp
import RobotControl.Serial_Servo_Running as SSR

# the second mission: dodge the trap
#end is set to 0 initially in main.py
def cross_trap(org_img, end):
    #movement initialization
    straight = False
    left = False
    left2 = False
    right = False
    right2 = False
#图像处理部分
    org_img_copy = cv2.resize(org_img[:, 0:580], (rp.r_w, rp.r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    color = judge_color()
    print(color)
    frame_green = cv2.inRange(frame_hsv, rp.lab_range[color][0], rp.lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    cv2.imshow('closed', closed)#显示闭运算结果
    contours, hierarchy = cv2.findContours(closed, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)  # 找出轮廓
    image_filled = 255 * np.ones(closed.shape, np.uint8)  # 用于识别色块的图像
       #寻找坑的轮廓
    areaMaxContour, area_max = getAreaMaxContour(contours, 50)
    if areaMaxContour is not None:
           rect = cv2.minAreaRect(areaMaxContour)
    image_line_detection = np.zeros(closed.shape, np.uint8)
    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)
    lines = cv2.HoughLinesP(image_line_detection[0:260], 1, np.pi / 180, 30, 300, 5)
     #调整转向部分
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