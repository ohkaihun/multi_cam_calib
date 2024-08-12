import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ChessBoard import detect_chessboard
import os
import cv2
# 使用 SciPy 的 optimize 模块求解
from scipy.optimize import minimize
def calculate_ray_air(K, keypoints2d):
    """
    计算雷达回波信号强度 Ray_air

    参数:
    K (numpy.ndarray): 矩阵K的逆
    p (numpy.ndarray): 向量p

    返回:
    float: 雷达回波信号强度 Ray_air
    """
    # 计算 K^-1 * p
    ray_airs=[]
    keypoints2d=np.array(keypoints2d,dtype=np.float64)
    for idx in range(len(keypoints2d)):
        u = float(keypoints2d[idx, 0])
        v = float(keypoints2d[idx, 1])
        uvpoint = np.array([u, v, 1]).reshape(-1, 1)
        K_inv_p = np.dot(np.linalg.inv(K), uvpoint)

        # 计算 ||K^-1 * p||_2
        norm_K_inv_p = np.linalg.norm(K_inv_p, ord=2)

        # 计算 Ray_air
        ray_air = K_inv_p / norm_K_inv_p
        ray_air[1]=-ray_air[1]
        ray_air[2]=-ray_air[2]
        ray_airs.append(ray_air)
    # ray_airs=np.array(ray_airs,dtype=np.float64)
    return ray_airs
def calculate_normal_line(points_glass):
    normals=[]
    for idx in range(len(points_glass)):
        point_glass=points_glass[idx,:]
        norm_point_glass= np.linalg.norm(point_glass, ord=2)
        normal=point_glass/norm_point_glass
        normals.append(normal*-1)
    return normals
def calulate_refraction_Ray(ray_airs,normal_vectors,ni,nr):
    refraction_vectors=[]
    for idx in range(len(ray_airs)):
        theta_i = np.arccos(-np.dot(ray_airs[idx].T, normal_vectors[idx]))  # incident angle
        sin_theta_r = (ni / nr) * np.sin(theta_i)
        theta_r = np.arcsin(sin_theta_r)
        r = (ni / nr) * ray_airs[idx] + ((ni / nr) * np.cos(theta_i) - np.sqrt(1 - (np.sin(theta_i)) ** 2) )* normal_vectors[idx]
        norm_r= np.linalg.norm(r, ord=2)
        r_ray=r/norm_r
        refraction_vectors.append(r_ray)
    return refraction_vectors
def normalize(v):
    """
    对向量v进行归一化

    参数:
    v (numpy.ndarray): 待归一化的向量

    返回:
    numpy.ndarray: 归一化后的向量
    """
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def calculate_theta(i, n):
    """
    计算 θi = arccos(-i · n)

    参数:
    i (numpy.ndarray): 向量i
    n (numpy.ndarray): 向量n

    返回:
    float: θi 的值
    """
    # 对i和n进行归一化
    i_norm = normalize(i)
    n_norm = normalize(n)

    # 计算 -i · n
    dot_product = -np.dot(i_norm, n_norm)

    # 计算 θi = arccos(-i · n)
    theta = np.arccos(dot_product)

    return theta
def calulate_point_intersection(ray_airs,offset,R):
    points_glass=[]
    x0=offset[0,0]
    y0=offset[1,0]
    z0=offset[2,0]
    for idx in range(len(ray_airs)):
        dx = ray_airs[idx][0, 0]
        dy = ray_airs[idx][1, 0]
        dz = ray_airs[idx][2, 0]
        # 求解二次方程
        a = np.roots([dx ** 2 + dy ** 2 + dz ** 2,
                      2 * (x0 * dx + y0 * dy + z0 * dz),
                      x0 ** 2 + y0 ** 2 + z0 ** 2 - R ** 2])
        for i in range(len(a)):
            if a[i]>0:
                lambda_=a[i]
            else:
                continue
        # 计算交点坐标
        x = x0 + lambda_ * dx
        y = y0 + lambda_ * dy
        z = z0 + lambda_ * dz
        points_glass.append(np.array([x,y,z]).reshape(3,1))
    return points_glass

ni=1
nr=1.33
root1Path=r"C:\Users\Mayn\work\calibration\LED\blender\results\1200_dome_cube"
root2Path=r"C:\Users\Mayn\work\calibration\LED\blender\results\1200_wo_dome_cube"
imgName=r'image_08.png'
outPath=r"C:\Users\Mayn\work\calibration\LED\blender\results"
board_dict = {
    'dict': 'cv2.aruco.DICT_4X4_250',
    'row': 4,
    'col': 6,
    'marker_size': 15,
    'square_size': 30}
gridSize=0.1968
ext='.png'
num_board=8
K=np.array([[400,0,600],[0,400,600],[0,0,1]])
d=1#1m
offset=np.array([[-0.5,-0.3, -0.1]]).reshape(3,1)
# # 计算变换矩阵 H
# H = np.array([[d, 0, 0, offset[0]],
#               [0, d, 0, offset[1]],
#               [0, 0, d, offset[2]],
#               [0, 0, 0, -1]])
#
# # 计算 D 矩阵
# D = np.linalg.inv(H).T @ np.diag([1, 1, 1, -1])@np.linalg.inv(H)
# # 定义目标函数
#R是c2w,旋转矩阵的第一列到第三列分别表示了相机坐标系的X, Y, Z轴在世界坐标系下对应的方向；平移向量表示的是相机原点在世界坐标系的对应位置。T是W2C相应的旋转向量
R = [[1,0,0],[0,-1,0],[0,0,-1]]
C = [[-0.5,-0.3, -0.1]]
R=np.array(R,dtype=np.float64).reshape((3,3))
C=np.array(C,dtype=np.float64).reshape((3,1))
T=- R@C
dist=np.zeros((5,1))





#air-dome-refraction
detect_flag,pointData=detect_chessboard(os.path.join(root1Path, imgName), outPath, "Intri", board_dict, gridSize, ext,True,False,num_board,None,None)
keypoints3d=pointData['keyPoints3d_1']
keypoints2d=pointData['keyPoints2d_1']
ray_airs=calculate_ray_air(K,keypoints2d)
points_glass=calulate_point_intersection(ray_airs,offset,d)
points_glass=np.array(points_glass)
normal_vectors=calculate_normal_line(points_glass)
refraction_vectors=calulate_refraction_Ray(ray_airs,normal_vectors,ni,nr)




#wo dome only air
detect_flag,pointData=detect_chessboard(os.path.join(root2Path, imgName), outPath, "Intri", board_dict, gridSize, ext,True,False,num_board,None,None)
k3d=np.array(pointData['keyPoints3d_1'])
k2d=np.array(pointData['keyPoints2d_1'])
#r,t W2C
ret, rvec, tvec = cv2.solvePnP(k3d, k2d, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
rvec_board = cv2.Rodrigues(rvec)[0]
tvec_board = tvec
points_board=[]
for p in range(board_dict['col']*board_dict['row']):
    u = float(k2d[p, 0])
    v = float(k2d[p, 1])
    # calculata s
    uvpoint = np.array([u, v, 1]).reshape(-1, 1)
    leftSideMat = np.linalg.inv(rvec_board) @ np.linalg.inv(K) @ uvpoint
    rightSideMat = np.linalg.inv(rvec_board) @ tvec_board
    s = (0 + rightSideMat[2, 0]) / leftSideMat[2, 0]
    # prejection form 2d to 3d
    # X is the 3d point in the chessboard coordinate system(with z=0) and Pworld is the 3d point in the camera coordinate system
    X = np.linalg.inv(rvec_board) @ ((s * np.linalg.inv(K) @ uvpoint) - tvec_board)
    P_world = np.dot(R, np.dot(rvec_board, X) + tvec_board -T)
    points_board.append(P_world)
points_board=np.array(points_board)




# 创建 3D 图形
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 绘制点和标注序号
for i in range(points_glass.shape[0]):
    ax.text(points_glass[i,0,0], points_glass[i,1,0], points_glass[i,2,0], str(i+1), color='k', size=10, zorder=1)
    ax.scatter(points_glass[i,0,0], points_glass[i,1,0], points_glass[i,2,0], s=50, c='g')
for i in range(points_board.shape[0]):
    ax.text(points_board[i,0,0], points_board[i,1,0], points_board[i,2,0], f'b_{i+1:02d}', color='k', size=10, zorder=1)
    ax.scatter(points_board[i,0,0], points_board[i,1,0], points_board[i,2,0], s=50, c='r')
for i in range(points_glass.shape[0]):
    # 计算射线的终点坐标
    x_end = points_glass[i,0,0] + 2 * refraction_vectors[i][0,0]
    y_end = points_glass[i,1,0] + 2 * refraction_vectors[i][1,0]
    z_end = points_glass[i,2,0] + 2 * refraction_vectors[i][2,0]
    # 绘制射线
    ax.plot([points_glass[i,0,0], x_end], [points_glass[i,1,0], y_end],[points_glass[i,2,0],z_end] ,color='blue')

# 绘制更精细的半球面
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi , 100)
x = np.outer(np.cos(u), np.sin(v))
y = np.outer(np.sin(u), np.sin(v))
z = np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_surface(x, y, z, rstride=1, cstride=1, color='b', alpha=0.2)

# 设置坐标轴范围和标签
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()
print(1)