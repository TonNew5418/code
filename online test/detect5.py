import cv2


def detect(readimg):
    readimg = cv2.cvtColor(readimg, cv2.COLOR_BGR2GRAY)
    readimg = readimg[:, 80:560]
    readimg = cv2.GaussianBlur(readimg, (5, 5), sigmaX=0)



    readimg = cv2.Canny(readimg, 80, 140)


    # cv2.imshow('1', readimg)
    # cv2.waitKey()


    circles = cv2.HoughCircles(readimg, cv2.HOUGH_GRADIENT, 1, 50, param1=50, param2=23, minRadius=15, maxRadius=25)

    if circles is not None:
        return circles[0, 0, 2], circles[0, 0, 0] + 80, circles[0, 0, 1]
    else:
        return 0, 0, 0
    
