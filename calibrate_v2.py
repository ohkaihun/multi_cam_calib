from typing import *
import os,json
from scipy.optimize import least_squares
from FileUtils import save_json, read_json, getFileList, makeDir,getFileList_aligned
import math
import g2o
import struct,random
import numpy as np
import cv2
from ChessBoard import detect_chessboard
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd #install tabulate

##intri
# 寻找单应矩阵，计算重投影误差
def reProjection(pointData,is_charu,num_board):
    if is_charu:
        error=0
        for board_index in range(num_board) :
            if f'mask_{board_index}' not in pointData:
                continue
            pointDetect = np.array(pointData[f'keyPoints2d_{board_index}'], dtype=np.float32)
            pointBoard = np.array(pointData[f'keyPoints3d_{board_index}'], dtype=np.float32)[:, :2]
            transMat, __ = cv2.findHomography(pointDetect, pointBoard, cv2.RANSAC, 1.0)
            # 使用perspectiveTransform时，需要注意，二维变三维， 整形转float型
            pointAfter = cv2.perspectiveTransform(pointDetect.reshape(1, -1, 2), transMat)
            pointAfter=  np.squeeze(pointAfter, axis=0)
            error+=np.sum((pointBoard-pointAfter)*(pointBoard-pointAfter))
            #print(pointBoard)
            #print(pointAfter)
            #print(pointBoard-pointAfter)
        return error
    else:
        pointDetect = np.array(pointData['keyPoints2d'], dtype=np.float32)
        pointBoard = np.array(pointData['keyPoints3d'], dtype=np.float32)[:, :2]
        transMat, __ = cv2.findHomography(pointDetect, pointBoard, cv2.RANSAC, 1.0)
        # 使用perspectiveTransform时，需要注意，二维变三维， 整形转float型
        pointAfter = cv2.perspectiveTransform(pointDetect.reshape(1, -1, 2), transMat)
        return np.sum((pointBoard - pointAfter) * (pointBoard - pointAfter))

def reprojection_selection(keypoint3d_selected,keypoint2dj_selected,Kj,R_saved,T_saved,dist):
    """
    Compute the reprojection error for a set of selected 3D keypoints and their corresponding 2D projections.

    Parameters:
    keypoint3d_selected (numpy.ndarray): A Nx3 numpy array containing the 3D coordinates of the selected keypoints.
    keypoint2dj_selected (numpy.ndarray): A Nx2 numpy array containing the 2D coordinates of the corresponding selected keypoints.
    Kj (numpy.ndarray): A 3x3 numpy array representing the camera intrinsic matrix.
    R_saved (numpy.ndarray): A 3x3 numpy array representing the camera rotation matrix.
    T_saved (numpy.ndarray): A 3x1 numpy array representing the camera translation vector.

    Returns:
    numpy.ndarray: A 1D numpy array containing the reprojection error for each keypoint.
    """
    # Compute the reprojection of the 3D keypoints
    points2d_repro, xxx = cv2.projectPoints(keypoint3d_selected, R_saved, T_saved, Kj,dist)
    kpts_repro = points2d_repro.squeeze()
    err = np.linalg.norm(points2d_repro.squeeze() - keypoint2dj_selected, axis=1).mean()

    return err,kpts_repro


#得到单个相机的内参K和畸变系数D后，对数据去畸变
def undistort(img_path,out_path,K,D,is_fisheye,k0,dim2,dim3,scale=0.6,imshow=False):#scale=1,图像越小
    # DIM=[1200,1200]
    scale=0.8
    if k0 is None:
        k0=K
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == dim2[0] / dim2[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K=K.copy()
    scaled_K[0][0] = K[0][0] * scale  # The values of K is to scale with image dimension.
    scaled_K[1][1] = K[1][1] * scale  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    if is_fisheye:
        undistorted_img =cv2.fisheye.undistortImage(img,K,D,Knew=k0)
    else:
        undistorted_img= cv2.undistort(img, K, D, None)
    if imshow:
        cv2.imshow("undistorted", undistorted_img)
    cv2.imwrite(out_path, undistorted_img)
    return undistorted_img,scaled_K
def undistort_list_of_points(point_list, in_K, in_d):
    K = np.asarray(in_K)
    d = np.asarray(in_d)
    # Input can be list of bbox coords, poly coords, etc.
    # TODO -- Check if point behind camera?
    points_2d = np.asarray(point_list)

    points_2d = points_2d[:, 0:2].astype('float32')
    points2d_undist = np.empty_like(points_2d)
    points_2d = np.expand_dims(points_2d, axis=1)

    result = np.squeeze(cv2.fisheye.undistortPoints(points_2d, K, d))

    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    for i, (px, py) in enumerate(result):
        points2d_undist[i, 0] = px * fx + cx
        points2d_undist[i, 1] = py * fy + cy

    return points2d_undist
def get_inner_points(width,height,pts0,pts1,MASK_i):
    height_min = 0
    height_max = height
    width_min = 0
    width_max = width

    # 创建一个布尔掩码,筛选出坐标在范围内的点
    mask = (pts0[:, 1] >= height_min) & (pts0[:, 1] < height_max) & \
           (pts0[:, 0] >= width_min) & (pts0[:, 0] <width_max) & \
           (pts1[:, 1] >= height_min) & (pts1[:, 1] < height_max) & \
           (pts1[:, 0] >= width_min) & (pts1[:, 0] <width_max)

    # 使用掩码筛选出符合要求的点
    PTS0 = pts0[mask]
    PTS1 = pts1[mask]
    new_MASK_i  = np.array(MASK_i,dtype=int)
    new_MASK_i=new_MASK_i[mask]
    return  PTS0,PTS1,new_MASK_i


# def estimate_pose_from_chessboard(Rvecs,Tvecs,i,j):


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
def rotationMatrixToQuaternion(R: np.ndarray) -> np.ndarray:
    """Creates a quaternion from a rotation matrix defining a given orientation.

    Parameters
    ----------
    R : [3x3] np.ndarray
        Rotation matrix

    Returns
    -------
    q : [4x1] np.ndarray
        quaternion defining the orientation
    """
    u_q0 = np.sqrt((1 + R[0, 0] + R[1, 1] + R[2, 2]) / 4)  # the prefix u_ means unsigned
    u_q1 = np.sqrt((1 + R[0, 0] - R[1, 1] - R[2, 2]) / 4)
    u_q2 = np.sqrt((1 - R[0, 0] + R[1, 1] - R[2, 2]) / 4)
    u_q3 = np.sqrt((1 - R[0, 0] - R[1, 1] + R[2, 2]) / 4)

    q = np.array([u_q0, u_q1, u_q2, u_q3])

    if u_q0 == max(q):
        q0 = u_q0
        q1 = (R[2, 1] - R[1, 2]) / (4 * q0)
        q2 = (R[0, 2] - R[2, 0]) / (4 * q0)
        q3 = (R[1, 0] - R[0, 1]) / (4 * q0)

    if u_q1 == max(q):
        q1 = u_q1
        q0 = (R[2, 1] - R[1, 2]) / (4 * q1)
        q2 = (R[0, 1] + R[1, 0]) / (4 * q1)
        q3 = (R[0, 2] + R[2, 0]) / (4 * q1)

    if u_q2 == max(q):
        q2 = u_q2
        q0 = (R[0, 2] - R[2, 0]) / (4 * q2)
        q1 = (R[0, 1] + R[1, 0]) / (4 * q2)
        q3 = (R[1, 2] + R[2, 1]) / (4 * q2)

    if u_q3 == max(q):
        q3 = u_q3
        q0 = (R[1, 0] - R[0, 1]) / (4 * q3)
        q1 = (R[0, 2] + R[2, 0]) / (4 * q3)
        q2 = (R[1, 2] + R[2, 1]) / (4 * q3)

    q =[q0, q1, q2, q3]
    return q

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
            # if cam.id == 0:
            #     continue

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
            baseline = 0
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
            # if i%2==1:
            #     continue
            # Add edge from first camera to the point
            edge = g2o.EdgeProjectXYZ2UV()

            # edge = g2o.EdgeProjectPSI2UV()
            # edge.resize(3)

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

        # for i,observation in enumerate(self.observations): # Ikke sikker på at det her er rette syntax
        #     if i%2==0:
        #         continue
        #     # Add edge from first camera to the point
        #     edge = g2o.EdgeProjectXYZ2UV()
        #
        #     # edge = g2o.EdgeProjectPSI2UV()
        #     # edge.resize(3)
        #
        #     # 3D point
        #     edge.set_vertex(0, point_vertices[observation.point_id])
        #     # Pose of first camera
        #     edge.set_vertex(1, camera_vertices[observation.camera_id])
        #     # edge.set_vertex(2, camera_vertices[anchor])
        #
        #     edge.set_measurement(observation.point)
        #     edge.set_information(np.identity(2))
        #     edge.set_robust_kernel(g2o.RobustKernelHuber())
        #
        #     edge.set_parameter_id(0, observation.camera_id)
        #     optimizer.add_edge(edge)


        print('num vertices:', len(optimizer.vertices()))
        print('num edges:', len(optimizer.edges()))
        print('Performing full BA:')
        optimizer.initialize_optimization()
        optimizer.set_verbose(True)
        optimizer.optimize(1000000)


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
        optimizer.save("test.g2o");

#初始化相机内参
def initIntriPara(outPath, camIds):
    intriPara = {}
    for camId in camIds:
        intriPara[camId] = read_json(os.path.join(outPath, camId + '.json'))
    return intriPara


def initIntriParam(paramPath,cams):##初始化所有相机的内参
    # paramList = os.listdir(paramPath)
    # paramList = [f for f in paramList if f.endswith('.json')]
    K = []
    dist = []
    for cam in cams:
        f = open(os.path.join(paramPath, cam+".json"),'r')
        dic = json.load(f)
        k = np.array(dic['K'], dtype=np.float32)
        d = np.array(dic['dist'], dtype=np.float32)
        new_k=np.array(dic['new_K'], dtype=np.float32)
        K.append(new_k)
        dist.append(d)
        f.close()
    return K,dist

#检测用于标定外参的某一帧是否有效
def isImageValid(imageName, outPath, camIds, ext):
    for camId in camIds:
        # print(os.path.join(outPath, "PointData", "Extri", camId, imageName.replace(ext, '.json')))
        if not os.path.exists(os.path.join(outPath, "PointData", "Extri", camId, imageName.replace(ext, '.json'))):
            return False
    return True

def estimate_pose(pts1, pts2,K1,K2,MASK_i):

    # # 基础矩阵
    # F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC,0.1)
    # # 选择inlier points
    # pts1 = pts1[mask.ravel() == 1]
    # pts2 = pts2[mask.ravel() == 1]
    # mask_inner=np.array(MASK_i,dtype=int)
    # mask_inner=mask_inner[mask.ravel()==1].tolist()
    # E = np.matmul(np.matmul(np.transpose(K2), F), K1)
    # retval, R, t, mask = cv2.recoverPose(E, pts1, pts2)
    # return pts1, pts2, mask_inner, R, t


    # # 使用基础矩阵和本质矩阵估计相机位姿
    E, mask = cv2.findEssentialMat(pts1, pts2, K1, cv2.RANSAC,threshold=0.1)
    # 选择 inlier points
    pts1_inner = pts1[mask.ravel() == 1]
    pts2_inner = pts2[mask.ravel()== 1]
    mask_inner=np.array(MASK_i,dtype=int)
    mask_inner=mask_inner[mask.ravel()==1].tolist()
    _, R, t, mask = cv2.recoverPose(E, pts1_inner, pts2_inner, K1)
    # _, R, t, _ = cv2.recoverPose(E, src_pts, dst_pts, K)

    #使用单应矩阵估计位姿
    # H = cv2.findHomography(pts1, pts2, method=cv2.RANSAC)
    # return pts1_inner,pts2_inner,mask_inner, R, t
    return R,t
# 三角化计算点云
def triangulate(pts1, pts2, R1, t1, R2, t2, K1, K2):
    # 将相机位姿转换为投影矩阵
    P1 = K1 @ np.hstack((R1, t1))
    P2 = K2 @ np.hstack((R2, t2))
    # 使用三角化计算匹配点的三维坐标
    points4D = cv2.triangulatePoints(P1, P2, pts1, pts2)
    points3D = cv2.convertPointsFromHomogeneous(points4D.T)# 将齐次坐标转化为欧式三维坐标
    return points3D.squeeze()

#光束平差
def bundle_adjustment(points3d, Ks, Rs, ts, point2dList,masklist,num_cam_pair):  #not used


    def get_start_end(masklist,index):
        start,end=0,0
        if index == 0:
            start = 0
            end = len(masklist[0])
        else:
            for k in range(index):
                start += len(masklist[k])
            end = start + len(masklist[index])
        return start,end
    """ 使用Bundle Adjustment优化三维点和相机位姿 """
    def reproj_error(params, camNum, Ks, point2dList,masklist):
        """ 重投影误差函数 """
        errors = []
        for i in range(camNum):
            R = params[i * 9:i * 9 + 9].reshape(3, 3)
            t = params[camNum * 9 + i * 3:camNum * 9 + i * 3 + 3].reshape(3, 1)
            if i==0:
                start1,end1=get_start_end(masklist,i)
                points3d1 = params[camNum * 12 + start1 * 3:camNum * 12 + end1 * 3]
                points3d2 = []
                points2d=point2dList[i]
            elif i==5:
                j=i-1
                start2,end2=get_start_end(masklist,j)
                points3d1 = []
                points3d2 = params[camNum * 12+start2*3:camNum * 12+end2*3]
                points2d=point2dList[2*i-1]
            else:
                j=i-1
                start1,end1=get_start_end(masklist,j)
                start2,end2=get_start_end(masklist,i)
                points3d1 = params[camNum * 12+start1*3:camNum * 12+end1*3]
                points3d2 = params[camNum * 12+start2*3:camNum * 12+end2*3]
                points2d = np.concatenate([point2dList[2*i-1], point2dList[2*i]])
            points3d= np.concatenate([points3d1,points3d2])
            points3d = points3d.reshape(-1,3)
            # 计算三维点在当前相机下的投影
            '''
            cv2.projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs[, imagePoints[, jacobian[, aspectRatio]]]) → imagePoints, jacobian
            objectPoints：三维物体坐标的数组，其形状为(N,1,3)，其中N是点的数目，每个点有三个坐标。
            rvec：旋转向量，是相机坐标系到物体坐标系的旋转向量。
            tvec：平移向量，是相机坐标系到物体坐标系的平移向量。
            cameraMatrix：相机矩阵，是相机内参的矩阵。
            distCoeffs：畸变系数，包含k1、k2、p1、p2、k3等系数。
            imagePoints：输出的二维图像坐标的数组，其形状为(N,1,2)。
            jacobian：可选的输出的导数矩阵，其形状为(N,2,6)。
            aspectRatio：可选的纵横比参数，用于调整相机内参矩阵中的焦距。
            返回值有两个，第一个是imagePoints是一个(N,1,2)形状的数组
            '''
            pointProj = cv2.projectPoints(points3d, R, t, Ks[i], None)[0]
            # 计算重投影误差
            errors.append(np.linalg.norm(pointProj.squeeze() - points2d.squeeze())) #squeeze去掉维度为1的维度
        return np.array(errors)

    camNum = len(Rs)
    # 将相机姿态和三维点坐标向量拼接
    Rs = np.array(Rs, dtype=np.float32)
    ts = np.array(ts, dtype=np.float32)
    points3d = np.concatenate([np.array(point).reshape(-1) for point in points3d])
    params = np.hstack((Rs.reshape(-1), ts.reshape(-1), points3d.reshape(-1)))
    # 使用scipy.optimize.least_squares优化重投影误差,第一个参数为计算残差的函数，第二个参数为自变量
    result = least_squares(reproj_error, params, args=(camNum, Ks, point2dList,masklist))
    # 将优化后的向量分解为相机位姿和三维点坐标
    Rs = result.x[:camNum * 9].reshape(camNum, 3, 3)
    ts = result.x[camNum * 9: camNum * 12].reshape(camNum, 3, 1)
    points3D = result.x[camNum * 12:].reshape(-1, 3)
    return points3D, Rs, ts

# 保存标定结果
def saveResult(outPath, Rs, Ts,cams):
    paramList = os.listdir(outPath)
    paramList = [f for f in paramList if f.endswith('.json')]
    for i, cam in enumerate(cams):
        f = open(os.path.join(outPath, cam+".json"), 'r')
        dic = json.load(f)
        dic['R'] = Rs[i].tolist()
        dic['t'] = Ts[i].tolist()
        f.close()
        f = open(os.path.join(outPath, cam+".json"), 'w')
        json.dump(dic, f, indent=4)
        f.close()

def saveResult_txt(outPath,camera_params):
    with open(outPath, "w") as f:
        f.write("# fu u0 v0 ar s | k1 k2 p1 p2 k3 | quaternion(scalar part first) translation | width height |\n")
        for params in camera_params:
            fu, u0, v0, ar, s, k1, k2, k3,k4, q0, q1, q2, q3, tx, ty, tz, width, height = params
            f.write(f"{fu:.6f} {u0:.6f} {v0:.6f} {ar:.6f} {s:.6f} | {k1:.6f} {k2:.6f}  {k3:.6f} {k4:.6f}| {q0:.6f} {q1:.6f} {q2:.6f} {q3:.6f} | {tx:.6f} {ty:.6f} {tz:.6f} {width} {height}\n")

    print("Output saved to", outPath)
def batch_triangulate(keypoints_, Pall, keypoints_pre=None, lamb=1e3):
    # keypoints: (nViews, nJoints, 3)
    # Pall: (nViews, 3, 4)
    # A: (nJoints, nViewsx2, 4), x: (nJoints, 4, 1); b: (nJoints, nViewsx2, 1)
    v = (keypoints_[:, :, -1]>0).sum(axis=0)
    valid_joint = np.where(v > 1)[0]
    keypoints = keypoints_[:, valid_joint]
    conf3d = keypoints[:, :, -1].sum(axis=0)/v[valid_joint]
    # P2: P矩阵的最后一行：(1, nViews, 1, 4)
    P0 = Pall[None, :, 0, :]
    P1 = Pall[None, :, 1, :]
    P2 = Pall[None, :, 2, :]
    # uP2: x坐标乘上P2: (nJoints, nViews, 1, 4)
    uP2 = keypoints[:, :, 0].T[:, :, None] * P2
    vP2 = keypoints[:, :, 1].T[:, :, None] * P2
    conf = keypoints[:, :, 2].T[:, :, None]
    Au = conf * (uP2 - P0)
    Av = conf * (vP2 - P1)
    A = np.hstack([Au, Av])
    if keypoints_pre is not None:
        # keypoints_pre: (nJoints, 4)
        B = np.eye(4)[None, :, :].repeat(A.shape[0], axis=0)
        B[:, :3, 3] = -keypoints_pre[valid_joint, :3]
        confpre = lamb * keypoints_pre[valid_joint, 3]
        # 1, 0, 0, -x0
        # 0, 1, 0, -y0
        # 0, 0, 1, -z0
        # 0, 0, 0,   0
        B[:, 3, 3] = 0
        B = B * confpre[:, None, None]
        A = np.hstack((A, B))
    u, s, v = np.linalg.svd(A)
    X = v[:, -1, :]
    X = X / X[:, 3:]
    # out: (nJoints, 4)
    result = np.zeros((keypoints_.shape[1], 4))
    result[valid_joint, :3] = X[:, :3]
    result[valid_joint, 3] = conf3d
    return result

def solvePnP(k3d, k2d, K, dist, flag, tryextri=False):
    k2d = np.ascontiguousarray(k2d[:, :2])
    # try different initial values:
    if tryextri:
        def closure(rvec, tvec):
            ret, rvec, tvec = cv2.solvePnP(k3d, k2d, K, dist, rvec, tvec, True, flags=flag)
            points2d_repro, xxx = cv2.projectPoints(k3d, rvec, tvec, K, dist)
            kpts_repro = points2d_repro.squeeze()
            err = np.linalg.norm(points2d_repro.squeeze() - k2d, axis=1).mean()
            return err, rvec, tvec, kpts_repro
        # create a series of extrinsic parameters looking at the origin
        height_guess = 2.1
        radius_guess = 7.
        infos = []
        for theta in np.linspace(0, 2*np.pi, 180):
            st = np.sin(theta)
            ct = np.cos(theta)
            center = np.array([radius_guess*ct, radius_guess*st, height_guess]).reshape(3, 1)
            R = np.array([
                [-st, ct,  0],
                [0,    0, -1],
                [-ct, -st, 0]
            ])
            tvec = - R @ center
            rvec = cv2.Rodrigues(R)[0]
            err, rvec, tvec, kpts_repro = closure(rvec, tvec)
            infos.append({
                'err': err,
                'repro': kpts_repro,
                'rvec': rvec,
                'tvec': tvec
            })
        infos.sort(key=lambda x:x['err'])
        err, rvec, tvec, kpts_repro = infos[0]['err'], infos[0]['rvec'], infos[0]['tvec'], infos[0]['repro']
    else:
        ret, rvec, tvec = cv2.solvePnP(k3d, k2d, K, dist, flags=flag)
        points2d_repro, xxx = cv2.projectPoints(k3d, rvec, tvec, K, dist)
        kpts_repro = points2d_repro.squeeze()
        err = np.linalg.norm(points2d_repro.squeeze() - k2d, axis=1).mean()
    # print(err)
    return err, rvec, tvec, kpts_repro
def extraPoint(dic_data,frame,num_board,pattern):
    board_mask=[]
    if dic_data[frame]['valid']==True:
        dic = dic_data[frame]
        points = np.zeros((0,2), dtype=np.float32)
        masks= []

        num_points_all=pattern[0]*pattern[1]
        for board_index in range(num_board):
            if f'mask_{board_index}' in dic:
                board_mask.append(board_index+ frame * num_board )
                coord = np.array(dic[f'keyPoints2d_{board_index}'], dtype=np.float32)
                mask=np.array(dic[f'mask_{board_index}'],dtype=int)+num_points_all*board_index
                points = np.vstack((points,coord))
                masks = np.concatenate((masks, mask), axis=0) # 拼接
            else:
                pass
        masks = np.array(masks)
        board_mask=np.array(board_mask)
    else:
        return False,None,None,None
    return True,points,masks,board_mask

def write_pointcloud(filename,xyz_points,rgb_points=None):

    """ creates a .pkl file of the point clouds generated
    """

    assert xyz_points.shape[1] == 3,'Input XYZ points should be Nx3 float array'
    if rgb_points is None:
        rgb_points = np.ones(xyz_points.shape).astype(np.uint8)*1
    assert xyz_points.shape == rgb_points.shape,'Input RGB colors should be Nx3 float array and have same size as input XYZ points'

    # Write header of .ply file
    fid = open(filename,'wb')
    fid.write(bytes('ply\n', 'utf-8'))
    fid.write(bytes('format binary_little_endian 1.0\n', 'utf-8'))
    fid.write(bytes('element vertex %d\n'%xyz_points.shape[0], 'utf-8'))
    fid.write(bytes('property float x\n', 'utf-8'))
    fid.write(bytes('property float y\n', 'utf-8'))
    fid.write(bytes('property float z\n', 'utf-8'))
    fid.write(bytes('property uchar red\n', 'utf-8'))
    fid.write(bytes('property uchar green\n', 'utf-8'))
    fid.write(bytes('property uchar blue\n', 'utf-8'))
    fid.write(bytes('end_header\n', 'utf-8'))

    CAM_RGB=0
    # Write 3D points to .ply file
    for i in range(xyz_points.shape[0]):
        # if i==xyz_points.shape[0]:
        #     fid.write(bytearray(struct.pack("fffccc", xyz_points[i, 0], xyz_points[i, 1], xyz_points[i, 2],
        #                                     CAM_RGB.tobytes(), CAM_RGB.tobytes(),
        #                                     CAM_RGB.tobytes())))
        # else:
        fid.write(bytearray(struct.pack("fffccc",xyz_points[i,0],xyz_points[i,1],xyz_points[i,2],
                                            rgb_points[i,0].tobytes() ,rgb_points[i,1].tobytes() ,
                                            rgb_points[i,2].tobytes() )))


    fid.close()

def plotpointsandpose(points_3d,R,t,K):
    '''
    plot points reprojection from cam_(index-1)
    plot camera pose from cam_(index)
    '''
    # 3D点数据
    # 可视化
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制3D点
    ax.scatter(points_3d[:, 0], points_3d[:, 1], points_3d[:, 2], c='b', s=10)

    # 绘制相机位姿
    camera_position = -R.T @ t
    ax.quiver(camera_position[0], camera_position[1], camera_position[2],
              R[0, 0], R[0, 1], R[0, 2], color='r', length=0.5)
    ax.quiver(camera_position[0], camera_position[1], camera_position[2],
              R[1, 0], R[1, 1], R[1, 2], color='g', length=0.5)
    ax.quiver(camera_position[0], camera_position[1], camera_position[2],
              R[2, 0], R[2, 1], R[2, 2], color='b', length=0.5)

    # 绘制3D点到相机的连线
    # for i in range(len(points_3d)):
    #     ax.plot([camera_position[0], points_3d[i, 0]],
    #             [camera_position[1], points_3d[i, 1]],
    #             [camera_position[2], points_3d[i, 2]], 'k--', linewidth=0.5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()
def get_points(point1, point2, mask1, mask2):
    # 取出 point1 和 point2 中对应的点
    common_indices = np.intersect1d(mask1, mask2)

    # 找到这些相同序号元素在 mask1 和 mask2 中的索引
    mask1_indices = [np.where(mask1 == idx)[0][0] for idx in common_indices]
    mask2_indices = [np.where(mask2 == idx)[0][0] for idx in common_indices]

    point1_selected = [point1[idx] for idx in mask1_indices]
    point2_selected = [point2[idx] for idx in mask2_indices]

    point1_selected = np.array(point1_selected,dtype=np.float32)
    point2_selected = np.array(point2_selected,dtype=np.float32)

    mask1_p=[mask1[idx] for idx in mask1_indices]
    mask2_p=[mask2[idx] for idx in mask2_indices]
    mask1_p = np.array(mask1_p,dtype=np.float32)
    mask2_p = np.array(mask2_p,dtype=np.float32)
    return point1_selected, point2_selected,mask1_indices,mask2_indices,mask1_p,mask2_p


def Unproject(points, Z, intrinsic, distortion):
    f_x = intrinsic[0, 0]
    f_y = intrinsic[1, 1]
    c_x = intrinsic[0, 2]
    c_y = intrinsic[1, 2]
    # This was an error before
    # c_x = intrinsic[0, 3]
    # c_y = intrinsic[1, 3]

    # Step 1. Undistort.
    points_undistorted = np.array([])
    if len(points) > 0:
        points_undistorted = cv2.undistortPoints(np.expand_dims(points, axis=1), intrinsic, distortion, P=intrinsic)
        points_undistorted = np.squeeze(points_undistorted, axis=1)

    # Step 2. Reproject.
    result = []
    for idx in range(points_undistorted.shape[0]):
        z = Z[0] if len(Z) == 1 else Z[idx]
        x = (points_undistorted[idx, 0] - c_x) / f_x * z
        y = (points_undistorted[idx, 1] - c_y) / f_y * z
        result.append([x, y, z])
    return result
def calibIntri(rootiPath,outPath, pattern, gridSize, ext, num_pic,is_charu,is_fisheye,num_board,num_cam):
    """
    rootiPath：data root path
    outPath：data output path
    pattern:Number of squares horizontally & vertically ,[4,6]
    gridSize:Square side length (in m)
    ext:Data extension ,'.png'
    num_pic:Number of pics for calibraion
    is_charu:Charucoboard detection
    is_fisheye:Fisheye cam system or pinhole cam
    num_board:Number of boards
    num_cam:Number of cameras
    calibrate intri & extri parameters
    """
    iFolder = getFileList_aligned(rootiPath, num_cam,num_pic,ext)
    camIds = [cam['camId'] for cam in iFolder]
    makeDir(outPath, camIds)
    pointcorner_data={}
    annots = {'cams': {
    }}
    Ks, Dists,Dists_perspective,Rvecs,Tvecs=[],[],[],[],[]
    ##Intri
    for cam in iFolder:
        dataList = []#per camera per frame data,include point2d_(board_index) ,point3d_(board_index),mask_(board_index)
        point_num=0
        valid_num=0  #num of valid pics used in calibration
        valid_num_undistorted=0
        for imgName in cam['imageList']:
            detect_flag,pointData=detect_chessboard(os.path.join(rootiPath, imgName), outPath, "Intri", pattern, gridSize, ext,is_charu,is_fisheye,num_board,None,None)
            if detect_flag==True:
                pointData['valid'] = True
                reProjErr = float(reProjection(pointData,is_charu,num_board))
                pointData['reProjErr'] = reProjErr
                valid_num+=1
                dataList.append(pointData)
            else:
                dataList.append(pointData)
                pointData['valid'] = False
                continue
        num = len(dataList)
        print("cam:",cam['camId'],"use:%02d images"%valid_num)
        if len(dataList) < num :
            print("cam:",cam['camId'], "calibrate intri failed: doesn't have enough valid image")
            continue
        dataList = sorted(dataList, key = lambda x: x['reProjErr'])
        i = 0
        for data in dataList:
            if i < num :
                data['selected'] = True
                i = i + 1
        point2dList=[]
        point3dList=[]
        for data in dataList:
            if data['selected'] == True & is_charu   :
                for board_index in range(num_board):
                    if f'mask_{board_index}'  in data:
                        point_num+=len(data[f'mask_{board_index}'])
                        point2dList.append(np.expand_dims(np.array(data[f'keyPoints2d_{board_index}'],dtype=np.float32),0))
                        point3dList.append(np.expand_dims(np.array(data[f'keyPoints3d_{board_index}'],dtype=np.float32),0))
                    else:
                        pass
            elif not data['selected'] == True& (is_charu):
                point2dList.append(np.expand_dims(np.array(data['keyPoints2d'], dtype=np.float32), 0))
                point3dList.append(np.expand_dims(np.array(data['keyPoints3d'], dtype=np.float32), 0))
            else:
                pass
        width=dataList[0]['iWidth']
        height=dataList[0]['iHeight']

        #do intri calibration using opencv2  cv2.calibrateCamera or  cv2.fisheye.calibrate
        if not is_fisheye:
            # 采用np.stack 对矩阵进行叠加,类型必须为float32，float64不可以
            ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(point3dList, point2dList, (width,height), None, None,flags=cv2.CALIB_FIX_K3)
        else:
            # point3dList=np.expand_dims(np.asarray(point3dList),1)
            # point2dList=np.expand_dims(np.asarray(point2dList),1)
            ret, K, dist, rvecs, tvecs = cv2.fisheye.calibrate(point3dList, point2dList,
                                                             (width, height), None,
                                                             None,flags = cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 1e-6))

        undistorted_dataList = []    #per camera per frame data,include point2d_(board_index) ,point3d_(board_index),mask_(board_index)
        #Undistort an image using the calibrated fisheye intrinsic parameters and distortion coefficients
        for imgName in cam['imageList']:
            img_path=os.path.join(os.path.join(rootiPath, imgName))
            out_path=os.path.join(os.path.join(outPath,'undistorted', imgName))
            undistorted_img,old_k=undistort(img_path,out_path,K,dist,is_fisheye,k0=None,dim2=[width,height],dim3=[ width,height])#dim2=[2448,2048],dim3=[ 2448,2048]
            detect_flag, pointData = detect_chessboard(os.path.join(outPath, 'undistorted', imgName),
                                                       outPath, "Intri_undistorted", pattern, gridSize, ext, is_charu,is_fisheye,
                                                       num_board,old_k,dist)
            # Undistort an image using the calibrated fisheye intrinsic parameters and distortion coefficients
            if detect_flag == True:
                pointData['valid'] = True
                reProjErr = float(reProjection(pointData, is_charu, num_board))
                pointData['reProjErr'] = reProjErr
                valid_num_undistorted += 1
                undistorted_dataList.append(pointData)
            else:
                pointData['valid'] = False
                undistorted_dataList.append(pointData)
                continue
        num = len(undistorted_dataList)
        i=0
        for data in undistorted_dataList:
            if i < num:
                i+=1
                data['selected'] = True
        print("cam:after distortion", cam['camId'], "use:%02d images" % valid_num_undistorted)
        point2dList_distorted = []
        point3dList_distorted = []
        for data in undistorted_dataList:
            if data['selected'] == True & is_charu:
                for board_index in range(num_board):
                    if f'mask_{board_index}' in data:
                        point_num += len(data[f'mask_{board_index}'])
                        point2dList_distorted.append(
                            np.expand_dims(np.array(data[f'keyPoints2d_{board_index}'], dtype=np.float32), 0))
                        point3dList_distorted.append(
                            np.expand_dims(np.array(data[f'keyPoints3d_{board_index}'], dtype=np.float32), 0))
                    else:
                        pass
            elif not data['selected'] == True & (is_charu):
                point2dList_distorted.append(np.expand_dims(np.array(data['keyPoints2d'], dtype=np.float32), 0))
                point3dList_distorted.append(np.expand_dims(np.array(data['keyPoints3d'], dtype=np.float32), 0))
            else:
                pass
        width = dataList[0]['iWidth']
        height = dataList[0]['iHeight']
        # do intri calibration using opencv2  cv2.calibrateCamera or  cv2.fisheye.calibrate
        # after distortion ,do calibration perspective camera model
        ret, K_distorted, dist_distorted, rvecs_distorted, tvecs_distorted = cv2.calibrateCamera(point3dList_distorted, point2dList_distorted,
                                                             (width, height), None,
                                                             None, flags=cv2.CALIB_FIX_K3,criteria=(
                                                               cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30,
                                                               1e-6))
        #save corner data used for extri calibration
        pointcorner_data[cam['camId']+'_Rvecs'] = rvecs_distorted
        pointcorner_data[cam['camId'] + '_Tvecs'] = tvecs_distorted
        pointcorner_data[cam['camId']] = undistorted_dataList
        #save intri K and D and RT(chessboard),this is used for inital CAMERA relative RT calibraion
        Ks.append(K_distorted)         #K_distorted should be the same as K
        Dists.append(dist) #5 parameters(k1,k2,p1,p2,k3) rather than 4 parameters(k1,k2,k3,k4)
        Dists_perspective.append(dist_distorted)
        #save results of calibation for visualation
        annots['cams'][cam['camId']] = {
            'K': K.tolist(),#fisheye K
            'D': dist.tolist(),
            'new_K':K_distorted.tolist(),#perspective K
            'new_dist':dist_distorted.tolist(),
            'width':width,
            'height':height
        }
    save_json(os.path.join(outPath, "cam_intri.json"), annots)
    return Ks,Dists,Dists_perspective,pointcorner_data,annots

def calib_initalExtri(num_pic,num_board,num_cam,pattern,outPath,Ks,Dists,Dists_perspective,pointcorner_data,annots):
    ##Extri
    KEYPOINTS2D = []
    MASK = []
    Init_parameters={}
    for cam in annots['cams'].keys():
        masks = []
        board_masks=[]
        points = np.zeros((0, 2), dtype=np.float32)
        # Find total keypoint2d(N*2),mask of keypoint2d(N*1),board_mask.  And concatentate them together eg.mask=[0,1,2,...,23,144,145] board_mask=[0,6],keypoints [[xxx,xxx],[yyy,yyy],...]
        for frame in range(num_pic):
            ret, keypoint2d, mask,board_mask = extraPoint(pointcorner_data[cam],frame, num_board,pattern)
            if not ret:
                continue
            points = np.vstack((points, keypoint2d))
            masks = np.concatenate((masks, mask + frame * num_board * pattern[0] * pattern[1]), axis=0)  # 拼接
            masks = np.array(masks, dtype=int)
            board_masks=np.concatenate((board_masks, board_mask ), axis=0)
        KEYPOINTS2D.append(points)
        MASK.append(masks)
        Init_parameters[cam+'_board_mask']=board_masks

    # Rs Ts SAVE Relative pose
    # R_abs T_abs SAVE Absolute pose
    Rs = []  # 各个相机旋转矩阵,0号相机的旋转矩阵为I，平均向量为0
    Rs.append(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32))
    Ts = []  # 各个相机平移
    Ts.append(np.array([[0], [0], [0]], dtype=np.float32))
    R_abs = [Rs[0]]
    T_abs = [Ts[0]]

    #only used for round-view camera system
    for index in range(num_cam):
        i = index % num_cam  # first cam index
        j = (index + 1) % num_cam # adjacent cam index
        #取出相机i和相机j的坐标和mask
        KEYPOINTS2D_i = KEYPOINTS2D[i]
        KEYPOINTS2D_j = KEYPOINTS2D[j]
        mask_i = MASK[i]
        mask_j = MASK[j]

        #找到公视点的2维坐标  以及其公视点下标
        keypoint2di_selected, keypoint2dj_selected, maski_selected, maskj_selected,maski_p,maskj_p= get_points(KEYPOINTS2D_i, KEYPOINTS2D_j,mask_i, mask_j)
        Ki=Ks[i]
        Kj=Ks[j]
        keypoint3d_selected=[]
        #initial extri
        common_indices = np.intersect1d(Init_parameters[f'cam{i:03d}_board_mask'], Init_parameters[f'cam{j:03d}_board_mask'])
        #公视棋盘板索引存在了board_selected里
        maski_indices = [np.where(Init_parameters[f'cam{i:03d}_board_mask'] == idx)[0][0] for idx in common_indices]

        for n,board_index in enumerate(common_indices):

            board_index=int(board_index)
            rvec_board = pointcorner_data[f'cam{i:03d}_Rvecs'][maski_indices[n]]
            rvec_board=cv2.Rodrigues(rvec_board)[0]
            tvec_board = pointcorner_data[f'cam{i:03d}_Tvecs'][maski_indices[n]]

            for m in range(board_index*pattern[0]*pattern[1],(board_index+1)*pattern[0]*pattern[1]):
                if m in maski_p:
                    p=np.where(maski_p==m)
                    u=float(keypoint2di_selected[p,0])
                    v=float(keypoint2di_selected[p,1])
                    #calculata s
                    uvpoint=np.array([u,v,1]).reshape(-1, 1)
                    leftSideMat=np.linalg.inv(rvec_board) @ np.linalg.inv(Ki) @ uvpoint
                    rightSideMat = np.linalg.inv(rvec_board)@ tvec_board
                    s = (0 + rightSideMat[2, 0]) / leftSideMat[2, 0]
                    #prejection form 2d to 3d
                    #X is the 3d point in the chessboard coordinate system(with z=0) and Pworld is the 3d point in the camera coordinate system
                    X=np.linalg.inv(rvec_board)  @ ((s*np.linalg.inv(Ki)@uvpoint) - tvec_board)
                    P_world = np.dot(np.linalg.inv(R_abs[i]), np.dot(rvec_board, X) + tvec_board- T_abs[i])
                    keypoint3d_selected.append(P_world)
                else:
                    continue
        points = np.array(keypoint3d_selected)

        #Connect camera i and camera j through the RT matrix obtained from the same chessboard
        #Select the set of relationships with the least error
        infos=[]
        for n, board_rt_selected in enumerate(common_indices):
            common_index = common_indices[n]
            # 由于pointcorner_data存RT的顺序和InitInit_parameters存板子序号的顺序一样，需要找到相机i和相机j对应的索引
            random_index1 = np.where(Init_parameters[f'cam{i:03d}_board_mask'] == common_index)[0][0]
            random_index2 = np.where(Init_parameters[f'cam{j:03d}_board_mask'] == common_index)[0][0]
            # 找到相应的rvec,tvec
            rvec1 = pointcorner_data[f'cam{i:03d}_Rvecs'][random_index1]
            tvec1 = pointcorner_data[f'cam{i:03d}_Tvecs'][random_index1]
            rvec2 = pointcorner_data[f'cam{j:03d}_Rvecs'][random_index2]
            tvec2 = pointcorner_data[f'cam{j:03d}_Tvecs'][random_index2]

            # 计算相机1的外参矩阵
            R1, _ = cv2.Rodrigues(rvec1)
            t1 = tvec1.squeeze()
            P1 = np.eye(4)
            P1[:3, :3] = R1
            P1[:3, 3] = t1

            # 计算相机2的外参矩阵
            R2, _ = cv2.Rodrigues(rvec2)
            t2 = tvec2.squeeze()
            P2 = np.eye(4)
            P2[:3, :3] = R2
            P2[:3, 3] = t2
            T_21 = np.dot(P2, np.linalg.inv(P1))
            R = T_21[:3, :3]
            t = T_21[:3, 3]
            R_saved = np.matmul(R, R_abs[i])
            T_saved = T_abs[i] + np.dot(R_abs[i], t)[:, None]

            err,kpts_repro=reprojection_selection(points,keypoint2dj_selected,Kj,R_saved,T_saved,Dists_perspective[j])
            infos.append({
                'err': err,
                'repro': kpts_repro,
                'R_saved': R_saved,
                'T_saved': T_saved
            })
        # infos.sort(key=lambda x:x['err']), the sorting is done in ascending order
        infos.sort(key=lambda x:x['err'])
        err, r_saved, t_saved, kpts_repro = infos[0]['err'], infos[0]['R_saved'], infos[0]['T_saved'], infos[0]['repro']
        print(err)
        R_abs.append(r_saved)
        T_abs.append(t_saved)

        ##save shared view point of camera i and j

        # keypoint3d_selected.append(np.array([0,0 ,0], dtype=np.float64).reshape(-1, 1))
        # points = np.array(keypoint3d_selected)
        # plotpointsandpose(points,R_abs[i],T_abs[i],Ks[i])
        # os.makedirs(os.path.join(outPath, 'pointcloud'), exist_ok=True)
        # filename = f'pointCloud_{index:02d}.ply'
        # output_filename = os.path.join(outPath, 'pointcloud', filename)
        # write_pointcloud(output_filename, points)

        keypoint3d_selected=np.array(keypoint3d_selected,dtype=float)

        Init_parameters[f'cam{index:02d}' + '_keypoint3d_selected'] = keypoint3d_selected
        Init_parameters[f'cam{index:02d}' + '_keypoint2di_selected'] = keypoint2di_selected
        Init_parameters[f'cam{index:02d}' + '_keypoint2dj_selected'] = keypoint2dj_selected
        Init_parameters[f'cam{index:02d}' + '_maski_p'] = maski_p
    Init_parameters['R_abs'] = R_abs
    Init_parameters['T_abs'] = T_abs
    return Init_parameters

def calib_Extri_BA(outPath,num_cam,Init_parameters):
    # Initialize map for BA
    map = Map()


    R_abs=Init_parameters['R_abs']
    T_abs=Init_parameters['T_abs']


    for index in range(num_cam):
        i = index % num_cam  # first cam index
        j = (index + 1) % num_cam # adjacent cam index
        maski_p =Init_parameters[f'cam{index:02d}' + '_maski_p']
        keypoint3d_selected=Init_parameters[f'cam{index:02d}' + '_keypoint3d_selected']
        keypoint2di_selected=Init_parameters[f'cam{index:02d}' + '_keypoint2di_selected']
        keypoint2dj_selected=Init_parameters[f'cam{index:02d}' + '_keypoint2dj_selected']
        # feed 3dpoint and corresponding 2dpoint of camera i and j to BA map
        for n, m in enumerate(maski_p):
            idx3d = int(m)
            map.points.append(Point(idx3d, keypoint3d_selected[n,:,:]))
            map.observations.append(Observation(idx3d, i, keypoint2di_selected.T[:, n]))
            map.observations.append(Observation(idx3d, j, keypoint2dj_selected.T[:, n]))

        # save length of masklist
        map.L.append(len(maski_p))
        #feed K R,T to map
        if index < 1:
            cam = Camera(index, R_abs[0], T_abs[0], True)  # 存下该相机的参数：R,T->POSE
        else:
            cam = Camera(index, R_abs[index], T_abs[index], False)  # 存下该相机的参数：R,T->POSE
        map.cameras.append(cam)  # 写入map
        map.K.append(Ks[index])

    print(
        f"Before BA reprojection error: {map.reproj_err()[0]:.2f}, reprojection error per observation :{map.reproj_err()[1]:.2f}")
    # Do bundle_adjustment
    map.bundle_adjustment()
    print(
        f"After BA reprojection error: {map.reproj_err()[0]:.2f}, reprojection error per observation :{map.reproj_err()[1]:.2f}")
    # save 3d points after BA
    points = []
    for i in range(len(map.points)):
        if map.points[i].id < 10000:
            points.append(np.array(map.points[i].point, dtype=np.float32))
        else:
            continue

    # save RT pose after BA
    camera_params=[]
    for i, cam in enumerate(annots['cams'].keys()):
        if i == 0:
            R = R_abs[0]
            T = T_abs[0]
        else:
            R = R_abs[i]
            T = T_abs[i]
        k = Ks[i]
        d = Dists[i]

        c2w = np.eye(4)
        c2w[:3, :3] = R
        c2w[:3, 3] = T.squeeze()
        annots['cams'][cam]['c2w_old'] = c2w

        c2w_ba= np.eye(4)
        c2w_ba[:3, :] = map.cameras[i].pose[:3, :]
        c2w_ba[:3,3]=c2w_ba[:3,3]
        annots['cams'][cam]['c2w_new'] = c2w_ba
        width=annots['cams'][cam]['width']
        height=annots['cams'][cam]['height']
        ##txt format
        # fu, u0, v0, ar, s, k1, k2, p1, p2, k3, q0, q1, q2, q3, tx, ty, tz
        fu = k[0][0]
        u0 = k[0][2]
        v0 = k[1][2]
        ar = 1
        s = k[0][1]
        k1 = float(d[0])
        k2 = float(d[1])
        k3 = float(d[2])
        k4 = float(d[3])
        q0, q1, q2, q3 = rotationMatrixToQuaternion(c2w[:3, :3])
        t1, t2, t3 = c2w[:3, 3].T
        camera_params.append([fu, u0, v0, ar, s, k1, k2, k3, k4, q0, q1, q2, q3, t1, t2, t3, width, height])



#TOTAL POINTS AFTER BA  save it.
    points = np.array(points)
    # print(points)
    os.makedirs(os.path.join(outPath, 'pointcloud'), exist_ok=True)
    filename = 'pointCloud.ply'
    output_filename = os.path.join(outPath, 'pointcloud', filename)
    write_pointcloud(output_filename, points)

    np.save(os.path.join(outPath, 'extri_annots.npy'), annots)
#ALIGN FORMAT.
    output_file = os.path.join(outPath, "camera_params.txt")
    saveResult_txt(output_file, camera_params)




if __name__ == '__main__':
    # torch.set_default_tensor_type('torch.cuda.FloatTensor')

    parser = argparse.ArgumentParser()
    parser.add_argument('--root_path', type=str, default='../sfm1/archive/bimage_fisheye_multicharuco_360')
    parser.add_argument('--out_path', type=str, default='../sfm1/archive/output_bimage_fisheye_multicharuco_360')
    parser.add_argument('--pattern', type=int, nargs='+', default=[4, 6])
    parser.add_argument('--gridsize', type=float,default=0.197)
    parser.add_argument('--ext', type=str,default='.png')
    parser.add_argument('--num_pic',type=int, default=18)
    parser.add_argument('--is_charu', default=False, action="store_true")
    parser.add_argument('--is_fisheye', default=False, action="store_true")
    parser.add_argument('--num_board', type=int, default=6)
    parser.add_argument('--num_cam', type=int, default=6)
    args = parser.parse_args()
    Ks, Dists, Dists_perspective,pointcorner_data,annots= calibIntri(args.root_path, args.out_path, args.pattern,args.gridsize,args.ext,args.num_pic,args.is_charu,args.is_fisheye,args.num_board,args.num_cam)
    Init_parameters=calib_initalExtri(args.num_pic,args.num_board,args.num_cam,args.pattern,args.out_path,Ks,Dists,Dists_perspective,pointcorner_data,annots)
    calib_Extri_BA(args.out_path,args.num_cam,Init_parameters)