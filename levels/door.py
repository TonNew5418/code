import cv2
import time
import numpy as np
import copy

import RobotControl.Serial_Servo_Running as SSR
from running_parameters import lab_range
from levels.necessary_functions import getAreaSecondMaxContour
from levels.necessary_functions import getAreaMaxContour


def check_stick(org_img, r_w, r_h):
    org_img_copy = cv2.resize(org_img.copy()[:, 0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
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
    org_img_copy = cv2.resize(org_img.copy()[:, 0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
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
        org_img_copy = cv2.resize(org_img.copy()[:, 0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
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


def get_angle(color, org_img, r_w=640, r_h=480):
    org_img_copy = copy.copy(org_img)
    org_img_copy = cv2.resize(org_img_copy, (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_green = cv2.inRange(frame_hsv, lab_range[color][0], lab_range[color][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    # result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    # cv2.imshow('closed', closed)

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


# 第5关：蓝色Π形门
def find_center_of_next(org_img, door_cnt, door_end, r_w=640, r_h=480):
    SSR.change_action_value("1", 1)
    time.sleep(0.5)

    # 调整对应的电机
    SSR.serial_setServo(20, 450, 500)
    time.sleep(0.5)
    SSR.serial_setServo(19, 550, 500)
    time.sleep(0.5)

    # 未检测到图像就休眠
    while org_img is None:
        time.sleep(0.5)
        print('delay 0.5s')

    org_img_copy = cv2.resize(org_img.copy()[:, 0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
    frame_gauss = cv2.GaussianBlur(org_img_copy, (3, 3), 0)  # 高斯模糊
    frame_hsv = cv2.cvtColor(frame_gauss, cv2.COLOR_BGR2Lab)  # 将图片转换到HSV空间
    frame_green = cv2.inRange(frame_hsv, lab_range['white'][0], lab_range['white'][1])  # 对原图像和掩模(颜色的字典)进行位运算
    opened = cv2.morphologyEx(frame_green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # 开运算 去噪点
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

    # 调试代码
    rgb_image = cv2.cvtColor(org_img_copy, cv2.COLOR_BGR2RGB)
    result = cv2.bitwise_and(rgb_image, rgb_image, mask=closed)
    # cv2.imshow('closed', closed)

    (contours, hierarchy) = cv2.findContours(closed[200:480, :], cv2.RETR_LIST,
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

    # 初始化图像线段检测矩阵
    image_line_detection = np.zeros(closed.shape, np.uint8)

    # 绘制检测到的所有边缘，颜色为白色，轮廓宽度为 2
    image_line_detection = cv2.drawContours(image_line_detection, areaMaxContour, -1, 255, 2)

    # 检测直线
    lines = cv2.HoughLinesP(image_line_detection[0:260], 1, np.pi / 180, 30, 300, 5)

    # 调整转向部分 #
    tangent_list = []
    length_list = []
    image_copy = copy.copy(org_img)

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 寻找不垂直且高度合适的直线
            if y1 < 250 and x2 - x1 != 0:
                tangent_abs = np.abs((y2 - y1) / (x2 - x1))
                # 寻找斜率很低的直线（水平直线）
                if tangent_abs < 0.5:
                    length_list.append(np.abs(line[0, 0] - line[0, 2]))  # 将线条长度添加到列表中
                    tangent_list.append([x1, y1, x2, y2])  # 将线条坐标添加到列表中

        # 如果有符合条件的直线
        if len(tangent_list) > 0:
            # 找到最长的直线
            [x1_f, y1_f, x2_f, y2_f] = tangent_list[np.where(length_list == np.max(length_list))[0][0]]
            # 将该直线斜率作为门框斜率
            tangent_used = -(y2_f - y1_f) / (x2_f - x1_f)  # 标准tangent

        # 如果没有符合条件的直线
        else:
            tangent_used = 0
            [x1_f, y1_f, x2_f, y2_f] = [0, 0, 0, 0]

        print('angle is: ', tangent_used)

        # cv2.line(image_copy, (x1_f, y1_f), (x2_f, y2_f), (0, 255, 0), 1)
        # cv2.line(image_line_detection, (x1_f, y1_f), (x2_f, y2_f), 255, 4)
        # cv2.circle(image_copy, (int(center[0]), int(center[1])), 5, (255, 0, 0), 2)
        # cv2.imshow('contour', image_line_detection)
        # cv2.imshow("image_line1", image_copy)

        # 根据斜率进行寻转调整
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
            frame_door = cv2.inRange(frame_hsv[20:120, :], lab_range['blue'][0],
                                     lab_range['blue'][1])  # 对原图像和掩模(颜色的字典)进行位运算
            opened_door = cv2.morphologyEx(frame_door, cv2.MORPH_OPEN, np.ones(( ), np.uint8))  # 开运算 去噪点
            closed_door = cv2.morphologyEx(opened_door, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  # 闭运算 封闭连接

            # 找出轮廓cv2.CHAIN_APPROX_NONE
            (contours_door, hierarchy_door) = cv2.findContours(closed_door, cv2.RETR_LIST,
                                                                           cv2.CHAIN_APPROX_NONE)

            print(door_cnt)

            # 如果没有检测到门的轮廓，就切换到下一个动作组
            if (contours_door is None) and door_cnt >= 0:
                print('go go go')
                time.sleep(2)
                # SSR.change_action_value("go_middle_change_3", 10)
                SSR.change_action_value("go_middle_stair", 10)
                time.sleep(7)
                door_end = 1

            # 如果检测到门的轮廓
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
                    print('cali_second')
                    # 利用上一次的最大面积进行二次校准判断门的轮廓
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
                            # SSR.change_action_value("go_middle_change_3", 9)
                            SSR.change_action_value("go_middle_stair", 10)
                            time.sleep(7)
                            door_end = 1
                            return door_cnt, door_end
                        # cv2.imshow("door line", door_line_detection)
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
                        return door_cnt, door_end
                    else:
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
                    return door_cnt, door_end

            cv2.imshow("closed_door", closed_door)
    else:
        print('no line')
        SSR.change_action_value("go_middle_stair", 2)
        time.sleep(2)
        return door_cnt, door_end


def door():
    while True:
        find_center_of_next()
        #     door_cali(320, 240)
        if door_end == 1:
            print('door end')
            time.sleep(2)
            find_area_percent()
            break


def find_area_percent(org_img, r_w, r_h):
    percent = 0
    straight_count = 0
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
            print('going out of mine', percent)
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
            straight_count = straight_count + 1
            if straight_count >= 3:
                break
