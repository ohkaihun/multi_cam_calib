import g2o
import numpy as np
from typing import *
import math,random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def quarternion_to_rotation_matrix(q):
    """
    The formula for converting from a quarternion to a rotation
    matrix is taken from here:
    https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    """
    qw = q.w()
    qx = q.x()
    qy = q.y()
    qz = q.z()
    R11 = 1 - 2 * qy ** 2 - 2 * qz ** 2
    R12 = 2 * qx * qy - 2 * qz * qw
    R13 = 2 * qx * qz + 2 * qy * qw
    R21 = 2 * qx * qy + 2 * qz * qw
    R22 = 1 - 2 * qx ** 2 - 2 * qz ** 2
    R23 = 2 * qy * qz - 2 * qx * qw
    R31 = 2 * qx * qz - 2 * qy * qw
    R32 = 2 * qy * qz + 2 * qx * qw
    R33 = 1 - 2 * qx ** 2 - 2 * qy ** 2
    R = np.array([[R11, R12, R13], [R21, R22, R23], [R31, R32, R33]])
    return R
class Point:
    """
    data structure:  id: point_id from 0-2k+
                     point: 3d dimension
    """
    def __init__(self, point_id: int, point: np.array):
        self.id = point_id
        self.point = point

    @property
    def x(self) -> float:
        return self.point[0]

    @property
    def y(self) -> float:
        return self.point[1]

    @property
    def z(self) -> float:
        return self.point[2]
class Observation:
    """
    data structure:
                     point_id :point_id  0-2k+
                     camera_id:camera_id  0-5
                     point: 2d dimension
    """
    def __init__(self, point_id: int, camera_id: int, point: np.array):
        self.point_id = point_id
        self.camera_id = camera_id
        self.point = point

    @property
    def u(self) -> float:
        return self.point[0]

    @property
    def v(self) -> float:
        return self.point[1]
class Camera:
    """
    data structure:
                     id:camera_id  0-5
                     R :np.array(3,3)
                     t :np.array(3,1)
                     fixed : true for camera0
    """
    def __init__(self, camera_id: int, R: np.array = np.eye(3), t: np.array = np.zeros(3), fixed: bool = False):
        self.id = camera_id
        self.R = R
        self.t = t
        self.fixed = fixed

    @property
    def pose(self) -> np.array:
        pose = np.eye(4)
        pose[:3, :3] = self.R
        pose[:3, 3] = self.t.ravel()
        return pose

    @pose.setter
    def pose(self, pose: np.array):
        self.R = pose[:3, :3]
        self.t = pose[:3, 3]
class Map:
    """
        points: List[Point]
        observations: List[Observation]
        cameras: List[Camera]
        K : List[Camera] np.array(6,3,3)
        L:List of masks
        true_poses:np.array(6,4,4)
    """
    def __init__(self):
        self.points: List[Point] = []
        self.observations: List[Observation] = []
        self.cameras: List[Camera] = []
        self.K : List[Camera]=[]
        self.L:List=[]
        self.true_poses=[]
    def remove_camera(self, cam_id: int):
        before = len(self.observations)
        self.cameras = [cam for cam in self.cameras if cam.id != cam_id]
        self.observations = [obs for obs in self.observations if obs.camera_id != cam_id]
        return before - len(self.observations)

    def invert_depth(self,x):
        assert len(x) == 3 and x[2] != 0
        return np.array([x[0], x[1], 1]) / x[2]
    def reproj_err(self) -> float:
        id2p = {p.id: p for p in self.points}
        id2c = {c.id: c for c in self.cameras}

        sum = 0
        for obs in self.observations:
            cam = id2c[obs.camera_id]
            p = id2p[obs.point_id].point
            q = np.append(p, 1)
            p_cam = cam.pose @ q
            t = self.K[cam.id] @ p_cam[:3]
            t /= t[2]
            dx = t[0] - obs.u
            dy = t[1] - obs.v
            err = dx ** 2 + dy ** 2
            err= math.sqrt(dx ** 2 + dy ** 2)

            sum += err
        return sum,sum/len(self.observations)

    def bundle_adjustment(self):
        '''
            revised from https://github.com/uoip/g2opy/blob/master/python/examples/ba_demo.py
        '''
        optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverEigenSE3()) #LinearSolverPCGSE3 LinearSolverDenseSE3
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        optimizer.set_algorithm(solver)

        # inliers = dict()
        camera_vertices = {}
        #define cam parameters
        for idx, camera in enumerate(self.cameras):
            focal_length = self.K[idx][0, 0]  #Only one parameter can be passed.
            principal_point = (self.K[idx][0, 2], self.K[idx][1, 2])
            baseline = 0.1
            cam = g2o.CameraParameters(focal_length, principal_point, baseline)
            cam.set_id(idx)
            optimizer.add_parameter(cam)

            # Use the estimated pose of the second camera based on the
            # essential matrix.
            pose = g2o.SE3Quat(camera.R, camera.t)
            self.true_poses.append(pose)
            # Set the poses that should be optimized.
            # Define their initial value to be the true pose
            # keep in mind that there is added noise to the observations afterwards.
            v_se3 = g2o.VertexSE3Expmap()
            v_se3.set_id(camera.id)
            v_se3.set_estimate(pose)
            print("fixed?", camera.fixed)
            v_se3.set_fixed(camera.fixed)
            optimizer.add_vertex(v_se3)
            camera_vertices[camera.id] = v_se3
            # print("camera id: %d" % camera.camera_id)
        print("num cam_vertices:", len(camera_vertices))
        anchor = 0
        point_vertices = {}
        #define 3d points
        for point in self.points:
            # Add 3d location of point to the graph
            vp = g2o.VertexPointXYZ()
            vp.set_id(point.id)
            vp.set_marginalized(True)
            # Use positions of 3D points from the triangulation

            # point_temp = np.array(point.point, dtype=np.float64) #TODO: forventer 3x1, er 4x1 (scale)
            point_temp = np.array(point.point[0:3], dtype=np.float64)

            vp.set_estimate(point_temp)
            optimizer.add_vertex(vp)
            point_vertices[point.id] = vp
        print("num point_vertices:", len(point_vertices))
        print("# observations:", len(self.observations))
        #define observation:image plane 2d
        for i,observation in enumerate(self.observations): # Ikke sikker på at det her er rette syntax

            edge = g2o.EdgeProjectXYZ2UV()

            # 3D point
            edge.set_vertex(0, point_vertices[observation.point_id])
            # Pose of first camera
            edge.set_vertex(1, camera_vertices[observation.camera_id])
            # edge.set_vertex(2, camera_vertices[anchor])

            edge.set_measurement(observation.point)
            edge.set_information(np.identity(2))
            edge.set_robust_kernel(g2o.RobustKernelHuber())

            edge.set_parameter_id(0, observation.camera_id)
            optimizer.add_edge(edge)


        print('num vertices:', len(optimizer.vertices()))
        print('num edges:', len(optimizer.edges()))
        print('Performing full BA:')
        optimizer.initialize_optimization()
        optimizer.set_verbose(True)
        optimizer.optimize(1000000)
        optimizer.save("test.g2o");

        for idx, camera in enumerate(self.cameras):
            print("Camera ID:", self.cameras[idx].id)
            t = camera_vertices[camera.id].estimate().translation()
            print("Camera Translation Before BA:\n", self.cameras[idx].t)
            self.cameras[idx].t = t
            print("Camera Translation After BA:\n", self.cameras[idx].t)
            q = camera_vertices[camera.id].estimate().rotation()
            print("Camera Rotation Before BA:\n", self.cameras[idx].R)
            self.cameras[idx].R = quarternion_to_rotation_matrix(q)
            print("Camera Rotation After BA:\n", self.cameras[idx].R)

        for idx, point in enumerate(self.points):
            p = point_vertices[point.id].estimate()
            # It is important to copy the point estimates.
            # Otherwise I end up with some memory issues.
            # self.points[idx].point = p
            # print("point before:", self.points[idx].point)
            self.points[idx].point = np.copy(p)
            # self.points[idx].point = np.hstack((self.points[idx].point, 1)) #fixes things

map=Map()
cam_num=2
Ks=[]
Rs=[]
ts=[]
# 相机1的内参
fx1 = 1000.0
fy1 = 1000.0
cx1 = 640.0
cy1 = 360.0
K1 = np.array([[fx1, 0, cx1],
               [0, fy1, cy1],
               [0, 0, 1]])

# 相机1的外参
R1 = np.array([
        [-4.37113882867379e-8, -4.37113882867379e-8, -1],
        [-1, 1.91068567692294e-15, 4.37113882867379e-8],
        [0, 1, -4.37113882867379e-8]])
t1 = np.array([0.2, .0, .0])

# 相机2的内参
fx2 = 1000.0
fy2 = 1000.0
cx2 = 640.0
cy2 = 360.0
K2 = np.array([[fx2, 0, cx2],
               [0, fy2, cy2],
               [0, 0, 1]])

# 相机2的外参
R2 = np.array([
        [0.866025388240814, -2.18556959197258e-8, -0.5],
        [-0.5, -3.78551732183041e-8, -0.866025388240814],
        [0, 1, -4.37113882867379e-8]])
t2 = np.array([0.100000001490116, 0.173205077648163, .0])

Ks.append(K1)
Ks.append(K2)
Rs.append(R1)
Rs.append(R2)
ts.append(t1)
ts.append(t2)
# 相机分辨率
width1 = 1280
height1 = 720
width2 = 1280
height2 = 720

# 随机生成三维点坐标,确保在两个相机可见范围内
num_points = 1000
points_3d = []
while len(points_3d) < num_points:
    X = random.uniform(-5, 5)
    Y = random.uniform(-5, 5)
    Z = random.uniform(-10, 10)
    p1 = np.dot(K1, np.dot(R1, [X, Y, Z]) + t1)
    u1 = p1[0] / p1[2]
    v1 = p1[1] / p1[2]
    p2 = np.dot(K2, np.dot(R2, [X, Y, Z]) + t2)
    u2 = p2[0] / p2[2]
    v2 = p2[1] / p2[2]
    if 0 <= u1 < width1 and 0 <= v1 < height1 and 0 <= u2 < width2 and 0 <= v2 < height2:
        points_3d.append([X, Y, Z])
points_3d=np.array(points_3d)
points_3d_gt = np.array(points_3d)

# 投影到两个相机的二维像素平面
points_2d_cam1 = []
points_2d_cam2 = []
for P in points_3d:
    p1 = np.dot(K1, np.dot(R1, P) + t1)
    u1 = p1[0] / p1[2]
    v1 = p1[1] / p1[2]
    points_2d_cam1.append([u1, v1])

    p2 = np.dot(K2, np.dot(R2, P) + t2)
    u2 = p2[0] / p2[2]
    v2 = p2[1] / p2[2]
    points_2d_cam2.append([u2, v2])

points_2d_cam1 = np.array(points_2d_cam1)
points_2d_cam2 = np.array(points_2d_cam2)
noise_std = 0.01
points_3d_noisy = points_3d + np.random.normal(0, noise_std, points_3d.shape)
print(points_3d_noisy-points_3d)
# 打印结果
for i, (P, p1, p2) in enumerate(zip(points_3d, points_2d_cam1, points_2d_cam2)):
    print(f"三维点坐标: {P}")
    print(f"相机1二维像素坐标: ({p1[0]:.2f}, {p1[1]:.2f})")
    print(f"相机2二维像素坐标: ({p2[0]:.2f}, {p2[1]:.2f})")

for n in range(num_points):
    idx3d = n
    map.points.append(Point(idx3d, points_3d_noisy[n,:]))
    map.observations.append(Observation(idx3d, 0, points_2d_cam1.T[:, n]))
    map.observations.append(Observation(idx3d, 1, points_2d_cam2.T[:, n]))

for index in range(cam_num):
    cam = Camera(index, Rs[index], ts[index])  # 存下该相机的参数：R,T->POSE
    map.cameras.append(cam)  # 写入map
    map.K.append(Ks[index])
print(
    f"Before BA reprojection error: {map.reproj_err()[0]:.2f}, reprojection error per observation :{map.reproj_err()[1]:.2f}")
# Do bundle_adjustment
map.bundle_adjustment()
print(
    f"After BA reprojection error: {map.reproj_err()[0]:.2f}, reprojection error per observation :{map.reproj_err()[1]:.2f}")
points_3d_pred=[]
for i in range(len(map.points)):
    points_3d_pred.append(np.array(map.points[i].point, dtype=np.float32))
points_3d_pred=np.array(points_3d_pred)
points_3d_dif=points_3d_pred-points_3d_gt


poses=[]
for i in range(cam_num):
    c2w_ba = np.eye(4)
    c2w_ba[:3, :] = map.cameras[i].pose[:3, :]
    c2w_ba[:3, 3] = c2w_ba[:3, 3]
    poses.append(c2w_ba)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
poses = np.array(poses)
# 计算相机位置的中心点
center = np.mean(poses[:, :3, 3], axis=0)

# 计算距离中心最远的相机位置
max_distance = 0
for i in range(poses.shape[0]):
    c2w = poses[i]
    camera_pos = c2w[:3, 3]
    print(camera_pos)
    distance = np.linalg.norm(camera_pos - center)
    if distance > max_distance:
        max_distance = distance
print(center, max_distance)
poses[:, :3, 3] = poses[:, :3, 3] - center  # 将所有相机的中心平移到原点

# 循环遍历每个相机的外参矩阵
for i in range(poses.shape[0]):
    # 获取当前相机的外参矩阵
    c2w = poses[i]

    # 相机位置
    camera_pos = c2w[:3, 3]
    # do normalization or not
    # camera_pos_normalized = camera_pos / max_distance
    camera_pos_normalized = camera_pos
    ax.scatter(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], color='r')

    ax.text(camera_pos_normalized[0] + 0.01, camera_pos_normalized[1] + 0.01, camera_pos_normalized[2] + 0.01,
            str(i), fontsize=8, color='black')

    # 相机坐标系中的xyz轴
    axes = c2w[:3, :3]
    ax.quiver(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], axes[0, 0], axes[1, 0],
              axes[2, 0], length=0.1, color='r')  # x 红色
    ax.quiver(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], axes[0, 1], axes[1, 1],
              axes[2, 1], length=0.1, color='g')  # y 绿色，朝向相机上方
    ax.quiver(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], axes[0, 2], axes[1, 2],
              axes[2, 2], length=0.1, color='b')  # z 蓝色，相机方向
# 设置坐标轴范围
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# 设置坐标轴标签
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 显示图形
plt.show()
# plt.savefig(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco\1.png")
