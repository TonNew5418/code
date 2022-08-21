import cv2
import copy
import numpy as np
import time
from levels.necessary_functions import getAreaMaxContour, judge_color
from running_parameters import r_h, r_w, lab_range
import running_parameters
import RobotControl.Serial_Servo_Running as SSR
import threading
import main


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



# divide the image into several parts
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

# 识别挡板的蓝色边框，判断正方向
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
    th1 = threading.Thread(target=get_img)
    th1.setDaemon(True)  # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
    th1.start()
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
        tangent_used = get_angle('blue')
        head_degree = 410 - forward_count * 3
        if head_degree < 380:
            head_degree = 380
        if move_count % 3 == 0:  # abled
            judge = True
            while judge:  # 通过识别挡板判断正方向
                print("head up")
                time.sleep(0.5)
                SSR.serial_setServo(20, head_degree, 1000)
                time.sleep(1)
                tangent_used = get_angle('blue')

                if tangent_used is not None:  # 向着蓝板转向，吧tangent_used的绝对值调整到0.07之内
                    if 1.20 > tangent_used > 0.07:
                        print("should turn left little")
                        SSR.change_action_value("turn_left", 1)
                        time.sleep(0.5)
                    elif tangent_used > 1.20:
                        print("should turn left large")
                        SSR.change_action_value("turn_left", 2)
                        time.sleep(0.5)
                    elif -0.07 > tangent_used > -1.20:
                        print("should turn right little")
                        SSR.change_action_value("turn_right", 1)
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

        OrgFrame = cv2.resize(org_img.copy()[:,0:580], (r_w, r_h), interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        # print(org_img)
        # cv2.imshow('init', OrgFrame)
        hsv = cv2.cvtColor(OrgFrame, cv2.COLOR_BGR2Lab)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 3)
        #Imask = cv2.inRange(hsv, color_dict['black_obstacle']['Lower'], color_dict['black_obstacle']['Upper'])
        Imask = cv2.inRange(hsv, lab_range['black'][0],lab_range['black'][1])
        # Imask = cv2.erode(Imask, None, iterations=2)
        Imask = cv2.dilate(Imask, np.ones((3, 3), np.uint8), iterations=2)

        n = -1
        obscle_list = []
        obscle_list_last = []
        # 识别每个区域是否有雷
        for r in roi_mine:
            n += 1
            blobs = Imask[r[0]:r[1], r[2]:r[3]]
            _, cnts, hierarchy = cv2.findContours(blobs, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            cnt_large, _ = getAreaMaxContour(cnts, 200)
            #####################################change 80 to 200##########################
            if cnt_large is not None:
                # 画出有雷的区域
                lefttop_x = r[2]
                lefttop_y = r[0]
                rightbottom_x = r[3]
                rightbottom_y = r[1]
                cv2.rectangle(OrgFrame, (lefttop_x, lefttop_y), (rightbottom_x, rightbottom_y), (0, 255, 255), 2)
                # cv2.imshow("what",org_img)
                # 雷区块的坐标分组
                obscle_list.append([n // 9, n % 9])
        s1 = [[5, 4], [5, 3], [5, 2], [5, 6], [5, 5], [4, 5], [4, 4], [4, 3], [3, 3], [3, 4], [3, 5]]
        s1_1 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7]]
        # 右前方有障碍
        s1_2 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 8]]
        # 左前方有障碍
        s1_3 = [[5, 4], [5, 3], [5, 2], [5, 1], [5, 5], [5, 6], [5, 7], [5, 0]]
        # 偏左方有障碍
        s2_1 = [[5, 1], [5, 0], [5, 3], [5, 2]]
        # 正中有障碍
        s2_2 = [[5, 4], [5, 5], [5, 6]]
        # 偏右方有障碍
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
        elif any(s in obscle_list for s in s_edge_left) and any(s in obscle_list for s in s1_2):
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
            print("head up")
            time.sleep(0.5)
            SSR.serial_setServo(20, head_degree, 1000)
            time.sleep(1)
            tangent_used = get_angle('blue')
            if tangent_used is not None:
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
    SSR.change_action_value("go_middle_stair", 4) # 保证机器人的脚靠在挡板上 
    time.sleep(5)
    cap.release()
    print('end of mine')


