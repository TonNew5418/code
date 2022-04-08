import os
import cv2
from IPython.display import Image, display
from detect1 import detect

def _test(img_name):
    # 获取路径
    img_path = os.path.join('images', img_name)
    save_path = os.path.join('outputs', img_name)

    # 读取图像
    img = cv2.imread(img_path)

    # 图像检测
    r, x, y = (int(x) for x in detect(img))

    # 绘制结果
    cv2.circle(img, (x, y), 3, (225, 0, 0), 5)
    cv2.circle(img, (x, y), r, (225, 0, 0), 3)

    # 保存并显示
    cv2.imwrite(save_path, img)
    display(Image(save_path))

if __name__ == '__main__':
    for i in range(1, 6):
        _test(str(i) + '.png')