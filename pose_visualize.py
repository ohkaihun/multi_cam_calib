import json
import numpy as np
import pandas as pd
# from tqdm import tqdm
import os
import cv2
import gzip
from FileUtils import  read_json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import argparse

# from data.scene import get_boundingbox
def read_npy(filepath):
    poses = []
    intrinsics = []
    near_fars = []
    w2cs = []
    file = np.load(filepath, allow_pickle=True).item()
    cams = file['cams']
    print(cams)
    for i in range(48):
        idx = f"{i:02d}"
        intrinsic = cams[idx]['K']
        extrinsic = cams[idx]['c2w']
        w2c = np.linalg.inv(extrinsic)
        intrinsics.append(intrinsic)
        poses.append(extrinsic)
        near_fars.append([1, 4.5])
        w2cs.append(w2c)
    return intrinsics, poses, near_fars, w2cs


def read_txt(filepath):
    poses = []
    for file in os.listdir(filepath):
        proj_mat_filename = os.path.join(filepath, file)
        intrinsic, extrinsic, near_far = read_cam_file(proj_mat_filename)
        # intrinsic[:2] *= 4  # * the provided intrinsics is 4x downsampled, now keep the same scale with image
        # intrinsic_matrices.append(intrinsic)
        poses.append(extrinsic)
    return poses


# def cal_scale_mat(self, img_hw, intrinsics, extrinsics, near_fars, factor=1.):
#     center, radius, _ = get_boundingbox(img_hw, intrinsics, extrinsics, near_fars)
#     radius = radius * factor
#     scale_mat = np.diag([radius, radius, radius, 1.0])
#     scale_mat[:3, 3] = center.cpu().numpy()
#     scale_mat = scale_mat.astype(np.float32)
#
#     return scale_mat, 1. / radius.cpu().numpy()
def read_cam_file(filename):
    with open(filename) as f:
        lines = [line.rstrip() for line in f.readlines()]
    # extrinsics: line [1,5), 4x4 matrix
    extrinsics = np.fromstring(' '.join(lines[1:5]), dtype=np.float32, sep=' ')
    extrinsics = extrinsics.reshape((4, 4))
    extrinsics = np.linalg.inv(extrinsics)
    # intrinsics: line [7-10), 3x3 matrix
    intrinsics = np.fromstring(' '.join(lines[7:10]), dtype=np.float32, sep=' ')
    intrinsics = intrinsics.reshape((3, 3))
    # depth_min & depth_interval: line 11
    depth_min = float(lines[11].split()[0])
    depth_max = depth_min + float(lines[11].split()[1]) * 192
    depth_interval = float(lines[11].split()[1])
    intrinsics_ = np.float32(np.diag([1, 1, 1, 1]))
    intrinsics_[:3, :3] = intrinsics
    return intrinsics_, extrinsics, [depth_min, depth_max]


# 修改过！！！！！

def load_K_Rt_from_P(filename, P=None):
    if P is None:
        lines = open(filename).read().splitlines()
        if len(lines) == 4:
            lines = lines[1:]
        lines = [[x[0], x[1], x[2], x[3]] for x in (x.split(" ") for x in lines)]
        P = np.asarray(lines).astype(np.float32).squeeze()

    out = cv2.decomposeProjectionMatrix(P)
    K = out[0]
    R = out[1]
    t = out[2]

    K = K / K[2, 2]
    intrinsics = np.eye(4)
    intrinsics[:3, :3] = K

    pose = np.eye(4, dtype=np.float32)
    pose[:3, :3] = R.transpose()  # ? why need transpose here
    pose[:3, 3] = (t[:3] / t[3])[:, 0]

    return intrinsics, pose  # ! return cam2world matrix here


def convert_ndc_to_pinhole(focal_length, principal_point, image_size):
    focal_length = np.array(focal_length)
    principal_point = np.array(principal_point)
    image_size_wh = np.array([image_size[1], image_size[0]])
    half_image_size = image_size_wh / 2
    rescale = half_image_size.min()
    principal_point_px = half_image_size - principal_point * rescale
    focal_length_px = focal_length * rescale
    fx, fy = focal_length_px[0], focal_length_px[1]
    cx, cy = principal_point_px[0], principal_point_px[1]
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
    return K
def align_extrinsic_params(true_extrinsic, relative_extrinsic):
    """
    将相对外参对齐到真实外参

    参数:
    true_extrinsic (list/ndarray): 真实的相机外参数据
    relative_extrinsic (list/ndarray): 相对的相机外参数据

    返回:
    aligned_extrinsic (list/ndarray): 对齐后的相机外参数据
    """
    aligned_extrinsic = []
    cam1_transform = np.linalg.inv(relative_extrinsic[0])@true_extrinsic[0]#
    aligned_extrinsic.append(cam1_transform)
    # 遍历每个相机的外参
    for i in range(1,len(true_extrinsic)):
        # 提取真实和相对外参的变换矩阵
        T_true = true_extrinsic[i]
        T_relative = relative_extrinsic[i]

        # 计算对齐变换矩阵
        T_align = np.dot(cam1_transform, T_relative)
        aligned_extrinsic.append(T_align)

    return np.array(aligned_extrinsic,dtype=float)


def calculate_rotation_error(cam1_extrinsic, cam2_extrinsic):
    """
    计算两组外参之间的旋转误差

    参数:
    cam1_extrinsic (list/ndarray): 第一组相机外参
    cam2_extrinsic (list/ndarray): 第二组相机外参

    返回:
    rotation_errors (list): 每个相机之间的旋转误差
    """
    rotation_errors = []
    for i in range(len(cam1_extrinsic)):
        R1 = cam1_extrinsic[i][:3, :3]
        R2 = cam2_extrinsic[i][:3, :3]

        # 计算旋转矩阵之间的差
        R_diff = np.dot(R1, R2.T)

        # 从旋转矩阵中提取旋转角度
        angle = np.arccos((np.trace(R_diff) - 1) / 2)
        rotation_errors.append(angle)

    return rotation_errors

def calculate_distance_error(cam1_extrinsic,cam2_extrinsic):
    distance_x = []
    distance_y = []
    distance_z = []
    for i in range(len(cam1_extrinsic)):
        T1 = cam1_extrinsic[i][:3, 3]
        T2 = cam2_extrinsic[i][:3, 3]
        difference = T1-T2
        distance_x.append(difference[0])
        distance_y.append(difference[1])
        distance_z.append(difference[2])
    return distance_x,distance_y,distance_z
# def opencv_from_cameras_projection(R, T, focal, p0, image_size):
#     R = torch.from_numpy(R)[None, :, :]
#     T = torch.from_numpy(T)[None, :]
#     focal = torch.from_numpy(focal)[None, :]
#     p0 = torch.from_numpy(p0)[None, :]
#     image_size = torch.from_numpy(image_size)[None, :]
#
#     R_pytorch3d = R.clone()
#     T_pytorch3d = T.clone()
#     focal_pytorch3d = focal
#     p0_pytorch3d = p0
#     T_pytorch3d[:, :2] *= -1
#     R_pytorch3d[:, :, :2] *= -1
#     tvec = T_pytorch3d
#     R = R_pytorch3d.permute(0, 2, 1)
#
#     # Retype the image_size correctly and flip to width, height.
#     image_size_wh = image_size.to(R).flip(dims=(1,))
#
#     # NDC to screen conversion.
#     scale = image_size_wh.to(R).min(dim=1, keepdim=True)[0] / 2.0
#     scale = scale.expand(-1, 2)
#     c0 = image_size_wh / 2.0
#
#     principal_point = -p0_pytorch3d * scale + c0
#     focal_length = focal_pytorch3d * scale
#
#     camera_matrix = torch.zeros_like(R)
#     camera_matrix[:, :2, 2] = principal_point
#     camera_matrix[:, 2, 2] = 1.0
#     camera_matrix[:, 0, 0] = focal_length[:, 0]
#     camera_matrix[:, 1, 1] = focal_length[:, 1]
#     return R[0], tvec[0], camera_matrix[0]


if __name__ == '__main__':
    # with open(r"C:\Users\DELL\Desktop\hot\transforms_train.json", 'r') as fp:
    #     meta = json.load(fp)
    # poses = []
    # for i in tqdm(meta['frames']):
    #     poses.append(i['transform_matrix'])
    # poses = np.array(poses).astype(np.float32) # 100×4×4
    # poses = np.load(r"D:\lab\project\sfm\result.npy")
    parser = argparse.ArgumentParser()
    parser.add_argument('--dirpath', type=str, default='../sfm1/archive/output_bimage_fisheye_multicharuco_360')
    args = parser.parse_args()
    dirpath = args.dirpath
    poses_old = []
    poses_ba=[]
    pose_gt=[]
    conversion_matrix = np.array([[-1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, -1, 0],
                                  [0, 0, 0, 1]])
    error_data = {
        'Cam': [0,1, 2,3,4,5]
    }
    frame_file_path = os.path.join(dirpath, "extri_annots.npy")
    frame_file=np.load(frame_file_path,allow_pickle=True).item()
    for i in range(len(frame_file['cams'])):
        camera=frame_file['cams'][f'cam{i:03d}']
        camera_pose=camera['c2w_old']
        camera_ba_pose=camera['c2w_new']
        poses_old.append(camera_pose)
        poses_ba.append(camera_ba_pose)
    # 创建一个3D图形
    #gt_pose
    gt_frame_root="../sfm1/archive/bimage_fisheye_multicharuco_360"
    for i in range (len(frame_file['cams'])):
        gt_file_path=os.path.join(gt_frame_root,f"{i:02d}",'transforms.json')
        gt_file=read_json(gt_file_path)
        gt_data=gt_file['frames'][0]['transform_matrix']
        gt_data=np.array(gt_data,dtype=float)
        gt_data[:3,3]=gt_data[:3,3]*1000#(m)->(mm)
        #transfrorm转换坐标系，X,Y,Z，opencv和blender坐标系不同
        gt_R=np.dot( gt_data,conversion_matrix)
        pose_gt.append(gt_R)
    # align pose to real world in blender
    aliged_pose=align_extrinsic_params(pose_gt,poses_old)
    # combine pose_gt and pose_predicted for visualization
    pose_gt = np.array(pose_gt)
    combined_pose = np.concatenate((pose_gt, aliged_pose), axis=0)
    #compute rot error and dis error
    rot_error=calculate_rotation_error(pose_gt,aliged_pose)
    dis_x,dis_y,dis_z=calculate_distance_error(pose_gt,aliged_pose)
    error_data['Rot. error (degree)']=rot_error
    error_data['Dis. x'] = dis_x
    error_data['Dis. y'] = dis_y
    error_data['Dis. z'] = dis_z
    #plot error
    df = pd.DataFrame(error_data)
    print(df.to_markdown(index=False))
    #plot figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    poses = np.array(combined_pose)
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
    print(center,max_distance)
    poses[:, :3, 3] = poses[:, :3, 3] - center  # 将所有相机的中心平移到原点

    # 循环遍历每个相机的外参矩阵
    for i in range(poses.shape[0]):
        # 获取当前相机的外参矩阵
        c2w = poses[i]

        # 相机位置
        camera_pos = c2w[:3, 3]
        #do normalization or not
        camera_pos_normalized = camera_pos / max_distance
        # camera_pos_normalized = camera_pos
        ax.scatter(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], color='r')

        ax.text(camera_pos_normalized[0] + 0.01, camera_pos_normalized[1] + 0.01, camera_pos_normalized[2] + 0.01,
                str(i), fontsize=8, color='black')

        # 相机坐标系中的xyz轴
        axes = c2w[:3, :3]
        ax.quiver(camera_pos_normalized[0], camera_pos_normalized[1], camera_pos_normalized[2], axes[0, 0], axes[1, 0],
                  axes[2, 0], length=0.1 , color='r'  )  # x 红色
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
