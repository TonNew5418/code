import math

import cv2


# 得到最大轮廓和对应的最大面积
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None
    # area_max_contour = np.array([[[0, 0]], [[0, 1]], [[1, 1]], [[1, 0]]])
    for c in contours:  # 遍历所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            # 只有在面积大于25时，最大面积的轮廓才是有效的，以过滤干扰
            if contour_area_temp > 25:
                area_max_contour = c
    # 返回最大的轮廓
    return area_max_contour, contour_area_max
