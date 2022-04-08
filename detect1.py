import os
from math import ceil

import cv2


def detect(readimg):

    readimg = cv2.cvtColor(readimg, cv2.COLOR_BGR2GRAY)
    readimg = cv2.medianBlur(readimg, 3)
    readimg = cv2.Canny(readimg, 200, 100)
    cv2.imshow('image', readimg)
    cv2.waitKey()
    circles = cv2.HoughCircles(readimg, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=20, minRadius=80, maxRadius=100)

    if circles is not None:
        return ceil(circles[0, 0, 2]), ceil(circles[0, 0, 0]), ceil(circles[0, 0, 1])
    else:
        return 0, 0, 0

if __name__ == '__main__':
    img_path = os.path.join('images', '2.png')
    img = cv2.imread(img_path)
    detect(img)
