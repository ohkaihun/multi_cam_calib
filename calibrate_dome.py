import argparse
import os
import cv2
import numpy as np
from calibrate_v2 import *
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
if __name__ == '__main__':
    # torch.set_default_tensor_type('torch.cuda.FloatTensor')

    parser = argparse.ArgumentParser()
    parser.add_argument('--root_path', type=str, default='../sfm1/archive/bimage_fisheye_multicharuco_360')
    parser.add_argument('--out_path', type=str, default='../sfm1/archive/output_bimage_fisheye_multicharuco_360')
    # parser.add_argument('--pattern', type=int, nargs='+', default=[4, 6])
    parser.add_argument('--dict', type=str, default='cv2.aruco.DICT_4X4_250')
    parser.add_argument('--row', type=int, default=6)
    parser.add_argument('--col', type=int, default=6)
    parser.add_argument('--marker_size', type=float, default=12.0)
    parser.add_argument('--square_size', type=float, default=16.0)
    parser.add_argument('--gridsize', type=float,default=0.197)
    parser.add_argument('--ext', type=str,default='.png')
    parser.add_argument('--num_pic',type=int, default=18)
    parser.add_argument('--is_charu', default=False, action="store_true")
    parser.add_argument('--is_fisheye', default=False, action="store_true")
    parser.add_argument('--num_board', type=int, default=6)
    parser.add_argument('--num_cam', type=int, default=6)
    parser.add_argument('--root_cam', type=int, default=0)
    args = parser.parse_args()
    board_dict={
            'dict': args.dict,
            'row':args.row,
            'col':args.col,
            'marker_size':args.marker_size,
            'square_size':args.square_size}
    annots={}
    pattern = (board_dict['row'], board_dict['col'])
    Ks,Dists, Dists_perspective,pointcorner_data,annots= calibIntri(args.root_path, args.out_path, board_dict,args.gridsize,args.ext,args.num_pic,args.is_charu,args.is_fisheye,args.num_board,args.num_cam)
    print(1)
    ni = 1
    nr = 1.33
    root1Path = r"C:\Users\Mayn\work\calibration\LED\blender\results\1648_dome_fisheye\00"
    root2Path = r"C:\Users\Mayn\work\calibration\LED\blender\results\1648_dome_fisheye"


    iFolder = getFileList_aligned(root1Path, args.num_cam, args.num_pic, args.ext)
    camIds = [cam['camId'] for cam in iFolder]
    makeDir(root2Path, camIds)
    width=pointcorner_data['cam000'][0]['iWidth']
    height=pointcorner_data['cam000'][0]['iHeight']

    for cam in iFolder:
        for imgName in cam['imageList']:
            img_path = os.path.join(os.path.join(root1Path, imgName))
            out_path = os.path.join(os.path.join(root2Path, 'undistorted', imgName))
            undistorted_img, old_k = undistort(img_path, out_path, Ks[0], Dists[0], args.is_fisheye, k0=None, dim2=[width, height],
                                               dim3=[width, height])  # dim2=[2448,2048],dim3=[ 2448,2048]
            detect_flag, pointData = detect_chessboard(os.path.join(root2Path, 'undistorted', imgName),
                                               root2Path, "Intri_undistorted", board_dict, args.gridsize,args.ext, args.is_charu,args.is_fisheye,
                                               args.num_board,old_k,Dists[0])
