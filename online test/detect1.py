import cv2


def detect(readimg):

    readimg = cv2.cvtColor(readimg, cv2.COLOR_BGR2GRAY)
    readimg = cv2.GaussianBlur(readimg, (3, 3), sigmaX=0)
    readimg = cv2.Canny(readimg, 150, 100)

    circles = cv2.HoughCircles(readimg, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=20, minRadius=20)

    if circles is not None:
        return circles[0, 0, 2], circles[0, 0, 0], circles[0, 0, 1]
    else:
        return 0, 0, 0
