import numpy as np
import g2o,random
class Camera:
    """
    data structure:
    """
    def __init__(self, camera_id: int, K: np.array = np.eye(3),pose: np.array= np.eye(4) ,fixed: bool = False):
        self.id = camera_id
        self.fx = K[0,0]
        self.fy=  K[1,1]
        self.cx= K[2,0]
        self.cy=K[2,1]
        self.fixed = fixed
        self.R=pose[:3,:3]
        self.t=pose[:3,3]
        self.pose=g2o.SE3Quat(self.R,self.t)
        self.baseline=0

class BundleAdjustment():
    def __init__(self, ):
        self.optimizer = g2o.SparseOptimizer()
        solver = g2o.BlockSolverSE3(g2o.LinearSolverCSparseSE3())
        solver = g2o.OptimizationAlgorithmLevenberg(solver)
        self.optimizer.set_algorithm(solver)

    def optimize(self, max_iterations=10):
        self.optimizer.initialize_optimization()
        self.optimizer.set_verbose(True)
        self.optimizer.optimize(max_iterations)
        self.optimizer.save("testgeo.g2o")
    def add_pose(self, pose_id, cam):
        sbacam = g2o.SBACam(cam.pose)
        sbacam.set_cam(cam.fx, cam.fy, cam.cx, cam.cy, cam.baseline)

        v_se3 = g2o.VertexCam()
        v_se3.set_id(pose_id * 2)   # internal id
        v_se3.set_estimate(sbacam)
        v_se3.set_fixed(cam.fixed)
        self.optimizer.add_vertex(v_se3)

    def add_point(self, point_id, point, fixed=False, marginalized=True):
        v_p = g2o.VertexSBAPointXYZ()
        v_p.set_id(point_id * 2 + 1)
        v_p.set_estimate(point)
        v_p.set_marginalized(marginalized)
        v_p.set_fixed(fixed)
        self.optimizer.add_vertex(v_p)

    def add_edge(self, point_id, pose_id,
            measurement,
            information=np.identity(2),
            robust_kernel=g2o.RobustKernelHuber(np.sqrt(5.991))):   # 95% CI

        edge = g2o.EdgeProjectP2MC()
        edge.set_vertex(0, self.vertex(point_id * 2 + 1))
        edge.set_vertex(1, self.vertex(pose_id * 2))
        edge.set_measurement(measurement)   # projection
        edge.set_information(information)

        if robust_kernel is not None:
            edge.set_robust_kernel(robust_kernel)
        self.optimizer.add_edge(edge)

    def get_pose(self, pose_id):
        return self.vertex(pose_id * 2).estimate()

    def get_point(self, point_id):
        return self.vertex(point_id * 2 + 1).estimate()


Ks=[]
Ps=[]
camlist=[]

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

P1 = np.eye(4)
P1[:3, :3] = R1
# 将 T 填入 P 的前三行第四列
P1[:3, 3] = t1.flatten()

P2 = np.eye(4)
P2[:3, :3] = R2
# 将 T 填入 P 的前三行第四列
P2[:3, 3] = t2.flatten()

cam=Camera(1,K1,P1,True)
camlist.append(cam)
cam=Camera(2,K2,P2,True)
camlist.append(cam)

Ks.append(K1)
Ks.append(K2)
Ps.append(P1)
Ps.append(P2)

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
noise_std = 100
points_3d_noisy = points_3d + np.random.normal(0, noise_std, points_3d.shape)
BA=BundleAdjustment()
for index in range(2):
    BA.add_pose(index,camlist[index])
for n in range(num_points):
    idx3d = n
    BA.add_point(idx3d,points_3d_noisy[n,:])
    BA.add_edge(idx3d,0,points_2d_cam1.T[:, n])
    BA.add_edge(idx3d,1,points_2d_cam2.T[:, n])
BA.optimize(1000)
