import cv2
import numpy as np


def detect(readimg):
    # 转换 HSV 色彩空间
    hsv = cv2.cvtColor(readimg, cv2.COLOR_BGR2HSV)

    # 通过 HSV 对应的绿色色彩区间对绿色部分进行提取
    lower = np.array([80, 43, 46])
    upper = np.array([95, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # 图像腐蚀 去除一些干扰小区域 只保留最大的绿色板
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # 获取坐标和长度
    index_x = np.where(mask.sum(0) > 0)[0]
    index_y = np.where(mask.sum(1) > 0)[0]
    offset = 5
    if len(index_x) and len(index_y):
        x0 = index_x[0] + offset
        x1 = index_x[-1] - offset
        y1 = index_y[-1] - offset
        return x1-x0, x0, y1
    else:
        return 0, 0, 0
