from math import ceil

import cv2


def detect(readimg):
    readimg = cv2.blur(readimg, (5, 5))
    readimg = cv2.Canny(readimg, 40, 80)
    circles = cv2.HoughCircles(readimg, cv2.HOUGH_GRADIENT, 1, 30, param1=50, param2=30, minRadius=80, maxRadius=100)

    if circles is not None:
        return ceil(circles[0, 0, 2]), ceil(circles[0, 0, 0]), ceil(circles[0, 0, 1])
    else:
        return 0, 0, 0
