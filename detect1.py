from math import ceil

import cv2


def detect(readimg):

    readimg = cv2.cvtColor(readimg, cv2.COLOR_BGR2GRAY)
    readimg = cv2.medianBlur(readimg, 3)
    readimg = cv2.Canny(readimg, 200, 100)

    circles = cv2.HoughCircles(readimg, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=20, minRadius=80, maxRadius=130)

    if circles is not None:
        return ceil(circles[0, 0, 2]), ceil(circles[0, 0, 0]), ceil(circles[0, 0, 1])
    else:
        return 0, 0, 0
