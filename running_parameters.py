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
