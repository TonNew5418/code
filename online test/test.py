import os
import cv2
from IPython.display import Image, display
from detect1 import detect

def _test(images_path, outputs_path):
    images_name = os.listdir(images_path)
    for image_name in images_name:
        img_path = os.path.join(images_path, image_name)
        save_path = os.path.join(outputs_path, (image_name.split('.')[0] + '.jpg'))

        try:
            img = cv2.imread(img_path)

            # 图像检测
            r, x, y = (int(x) for x in detect(img))

            # 绘制结果
            cv2.circle(img, (x, y), 3, (225, 0, 0), 5)
            cv2.circle(img, (x, y), r, (225, 0, 0), 3)

            # 保存并显示
            cv2.imwrite(save_path, img)
            display(Image(save_path))
        except Exception as e:
            print(e)

if __name__ == '__main__':
    _test('online test/images/20', 'outputs')
