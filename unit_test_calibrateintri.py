from calibrate_v2 import calibIntri
import random
import numpy as np
import cv2


import numpy as np

# 相机内参
K = np.array([[400, 0, 320],
              [0, 400, 240],
              [0, 0, 1]])
width=640
height=480
point3dList=[]
point2dList=[]
# 相机外参
R = np.eye(3)
t = np.array([0, 0, 2])
# 生成三维点数组
num_rows = 7
num_cols = 5
plane_width = 1.2  # 平面宽度为 4 米
plane_height = 0.8# 平面高度为 3 米
points_3d = []
point_chess=[]
for i in range(num_rows):
    for j in range(num_cols):
        x = (j - (num_cols - 1) / 2) * plane_width / (num_cols - 1)
        y = (i - (num_rows - 1) / 2) * plane_height / (num_rows - 1)
        x1=j* plane_width / (num_cols - 1)
        x2=i* plane_height / (num_rows - 1)
        point_chess.append(np.array([x2, x1, 0]))
        points_3d.append( np.dot(R, np.array([x2, x1,2])) + t)
# 计算二维点坐标
points_2d = []
for i in range(num_rows):
    for j in range(num_cols):
        pt_3d = points_3d[i* num_cols+j]
        p1 = np.dot(K, np.dot(R, pt_3d) + t)
        # pt_2d = np.dot(K, pt_3d)
        points_2d.append( p1[:2] / p1[2])
# points_2d=np.array(points_2d, dtype=np.float32)[::-1]
point3dList.append(np.expand_dims(np.array(point_chess, dtype=np.float32), 0))
point2dList.append(np.expand_dims(np.array(points_2d, dtype=np.float32), 0))
ret, K1, dist, rvecs, tvecs = cv2.calibrateCamera(point3dList, point2dList, (width,height), None, None,flags=cv2.CALIB_FIX_K3)
print(K,K1)