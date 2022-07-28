import cv2
import numpy as np

from CameraCalibration.CalibrationConfig import calibration_param_path

# 摄像头校准参数 #
param_data = np.load(calibration_param_path + '.npz')
assert param_data is not None, "加载参数失败, 检查 'calibration_param.npz' 路径"

dim = tuple(param_data['dim_array'])
k = np.array(param_data['k_array'].tolist())
d = np.array(param_data['d_array'].tolist())
print('摄像头校准参数加载完成')
print('dim:\n', dim)
print('k:\n', k)
print('d:\n', d)

# 优化内参和畸变参数 #
scale = 1
p = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(k, d, dim, None)
Knew = p.copy()
if scale:
    Knew[(0, 1), (0, 1)] = scale * Knew[(0, 1), (0, 1)]
map1, map2 = cv2.fisheye.initUndistortRectifyMap(k, d, np.eye(3), Knew, dim, cv2.CV_16SC2)

# 颜色参数 #
color_dict = {'red': {'Lower': np.array([0, 60, 60]), 'Upper': np.array([6, 255, 255])},
              'blue': {'Lower': np.array([100, 80, 46]), 'Upper': np.array([124, 255, 255])},
              # [(110, 43, 46), (124, 255, 255)]
              'green': {'Lower': np.array([35, 43, 46]), 'Upper': np.array([77, 255, 255])},
              'yellow_door': {'Lower': np.array([10, 43, 46]), 'Upper': np.array([34, 255, 255])},
              'black': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 46])},
              'black_obscle': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 60])},
              'black_dir': {'Lower': np.array([0, 0, 0]), 'Upper': np.array([180, 255, 46])}
              }

color_range = {'yellow_door': [(10, 43, 46), (34, 255, 255)],
               'red_floor1': [(0, 43, 46), (10, 255, 255)],
               'red_floor2': [(156, 43, 46), (180, 255, 255)],
               'green_bridge': [(35, 43, 20), (100, 255, 255)],
               'yellow_hole': [(10, 70, 46), (34, 255, 255)],
               'black_hole': [(0, 0, 0), (180, 255, 80)],
               'black_gap': [(0, 0, 0), (180, 255, 100)],
               'black_dir': [(0, 0, 0), (180, 255, 46)],
               'blue': [(110, 43, 46), (124, 255, 255)],
               'black_door': [(0, 0, 0), (180, 255, 46)],
               }

# 参数设置 #
cap = cv2.VideoCapture(-1)
Running = True
org_img = None  # 全局变量，原始图像
ret = False  # 读取图像标志位

debug = True
key = -1  # waitKey
isstop = True

# level2 parameters
step = 1
move_count = 0
forward_count = 0  # cross mine
end = 0
door_cnt = 0
door_end = 0

r_w = 640
r_h = 480
blue_persent = 0

lab_range = {
    # 'yellow_door': [(11, 63, 66), (34, 255, 255)],
    # 'black_door': [(0, 0, 0), (80, 255, 5)],
    #
    # 'blue': [(0, 0, 0), (255, 175, 94)], # dark blue
    # 'blue_bridge': [(137, 93, 0), (255, 122, 121)],  # light blue
    #
    # 'red': [(0, 154, 130), (255, 255, 255)],
    #
    # 'green_bridge': [(47, 0, 135), (255, 110, 255)],
    # 'black_hole': [(0, 0, 0), (180, 255, 80)],
    # 'black_gap': [(0, 0, 0), (180, 255, 100)],
    # 'black_dir': [(0, 0, 0), (180, 255, 46)],
    # 'blue_old': [(0, 92, 52), (255, 178, 111)],
    # 'blue_floor' : [(0, 0, 0), (255, 154, 102)],

    # 'white': [(215, 0, 0), (255, 255, 255)],

    'red': [(0, 154, 130), (255, 255, 255)],
    'green': [(47, 0, 125), (255, 110, 255)],
    'blue': [(0, 0, 0), (255, 175, 104)],  # 94
    'yellow': [(11, 63, 66), (34, 255, 255)],
    'white': [(215, 0, 0), (255, 255, 255)],
    'black': [(0, 0, 0), (50, 255, 255)]

}
lab_red = [(0, 154, 130), (255, 255, 255)]
