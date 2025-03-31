import cv2
import numpy as np
import os,pickle
from scipy.interpolate import griddata



def is_point_in_front_of_camera(camera_position, camera_rotation, point):
    V = point - camera_position  # 从相机到交点的向量
    D = -camera_rotation[:, 2]  # 相机的前方方向

    cos_theta = np.dot(V, D) / (np.linalg.norm(V) * np.linalg.norm(D))
    return cos_theta < 0  # 如果 cos(theta) < 0，表明在相机朝向内
def find_sphere_intersection(ray_origin, ray_direction,R,T):
    # 球的中心
    sphere_center = np.array([0, 0, -0.2])

    # 将射线起点移动到球心
    adjusted_origin = ray_origin - sphere_center

    a = np.dot(ray_direction, ray_direction)
    b = 2 * np.dot(adjusted_origin, ray_direction)
    c = np.dot(adjusted_origin, adjusted_origin) - 1  # 半径为1的单位球

    discriminant = b ** 2 - 4 * a * c
    if discriminant < 0:
        return None  # 没有交点

    t1 = (-b + np.sqrt(discriminant)) / (2 * a)
    t2 = (-b - np.sqrt(discriminant)) / (2 * a)

    intersection1 = ray_origin + t1 * ray_direction
    intersection2 = ray_origin + t2 * ray_direction
    if intersection1[2]>ray_origin[2]:
        return intersection1
    else:
        return intersection2


def invert_yz_of_rotation_matrix(R):
    # 定义 Y 和 Z 方向取反的对角矩阵
    D = np.diag([1, -1, -1])

    # 计算新的旋转矩阵
    R_new = np.dot(R, D)
    return R_new
import numpy as np

import numpy as np

def calculate_point_intersection(ray_airs, offset, r):
    # 提取 offset 的坐标
    x0, y0, z0 = offset[:, 0]

    # 从 ray_airs 中提取 dx, dy, dz
    dx = ray_airs[0, :]  # 第1行
    dy = ray_airs[1, :]  # 第2行
    dz = ray_airs[2, :]  # 第3行


    # 计算二次方程的系数
    a = dx**2 + dy**2 + dz**2
    b = 2 * (x0 * dx + y0 * dy + z0 * dz)
    c = x0**2 + y0**2 + z0**2 - r**2

    # 计算判别式
    discriminant = b**2 - 4 * a * c

    # 只在判别式非负时计算根
    root1 = (-b - np.sqrt(np.maximum(discriminant, 0))) / (2 * a)
    root2 = (-b + np.sqrt(np.maximum(discriminant, 0))) / (2 * a)

    # 选择正根
    lambda_positive = np.where(root1 > 0, root1, np.where(root2 > 0, root2, np.inf))

    # 计算交点坐标
    x_intersection = x0 + lambda_positive * dx
    y_intersection = y0 + lambda_positive * dy
    z_intersection = z0 + lambda_positive * dz

    # 将交点坐标组合成 (n, 3) 的数组
    intersections = np.vstack((x_intersection, y_intersection, z_intersection))

    return intersections
# def pixel_to_ray(uvpoint, K):
#     # 假设 K 是相机内参矩阵
#     # 这里的计算方式取决于具体的相机模型
#     # 返回光线方向
#     return np.linalg.inv(K) @ uvpoint
def pixel_to_ray(undistorted_points, K):
    """将像素坐标转换为光线方向。"""
    # 获取归一化坐标
    x = (undistorted_points[0,:] - K[0, 2]) / K[0, 0]
    y = (undistorted_points[1,:] - K[1, 2]) / K[1, 1]
    z = np.ones_like(x)
    ray_directions = np.vstack((x, y, z))
    # ray_directions[1:] = -ray_directions[1:]
    # ray_directions[2:] = -ray_directions[2:]
    ray_directions/=np.linalg.norm(ray_directions, axis=0, ord=2)
    return ray_directions
def undistort_points(uvpoints, K, dist_coeffs,w,h):
    # 将点从像素坐标转换为归一化坐标

    uvpoints = uvpoints[:, :2].reshape(-1, 1, 2)  # 27900 x 1 x 2
    normalized_points = cv2.undistortPoints(uvpoints, K, None, P=K)

    return normalized_points

def calculate_normal_line(points_glass):
    # 计算每个点的范数
    norms = np.linalg.norm(points_glass, axis=0, ord=2)  # axis=0 以列为单位计算

    # 计算法线，确保形状正确
    normals = points_glass / norms  # 使用广播

    # 反向法线
    normals *= -1

    return normals

def calculate_refraction_ray(ray_airs, normal_vectors, ni, nr):
    # 确保 ray_airs 和 normal_vectors 是单位向量
    ray_airs = ray_airs / np.linalg.norm(ray_airs, axis=0, keepdims=True)
    normal_vectors = normal_vectors / np.linalg.norm(normal_vectors, axis=0, keepdims=True)

    # 计算入射角的余弦值
    cos_theta_i = -np.einsum('ij,ij->j', ray_airs, normal_vectors)  # 点积
    cos_theta_i = np.clip(cos_theta_i, -1.0, 1.0)  # 限制在有效范围内
    theta_i = np.arccos(cos_theta_i)  # 入射角

    # 计算折射角的正弦值
    sin_theta_r = (ni / nr) * np.sin(theta_i)

    # 确保 sin_theta_r 在有效范围内
    sin_theta_r = np.clip(sin_theta_r, -1, 1)  # 确保不超过 [-1, 1]
    theta_r = np.arcsin(sin_theta_r)

    # 计算 cos(theta_r)
    cos_theta_r = np.sqrt(np.maximum(0, 1 - sin_theta_r ** 2))  # 计算 cos(theta_r)

    # 计算折射向量
    r = (ni / nr) * ray_airs + ((ni / nr) * cos_theta_i - cos_theta_r) * normal_vectors

    # 归一化折射向量
    norm_r = np.linalg.norm(r, axis=0, keepdims=True)
    r_ray = r / norm_r

    return r_ray
def interpolate_points_from_img(unique_coords, averaged_new_u, averaged_new_v):
    min_row, min_col = unique_coords.min(axis=0)
    max_row, max_col = unique_coords.max(axis=0)

    # 创建新的矩阵大小
    new_shape = (max_row - min_row + 1, max_col - min_col + 1)
    new_matrix_u = np.full(new_shape, np.nan)  # 用 NaN 填充以便于插值
    new_matrix_v = np.full(new_shape, np.nan)

    # combined_matrix = np.nan_to_num(combined_matrix, nan=0).astype(np.uint8)



    # 将 unique_coords 填充到新矩阵中
    for coord, u_val, v_val in zip(unique_coords, averaged_new_u, averaged_new_v):
        row, col = coord
        new_matrix_u[row - min_row, col - min_col] = u_val
        new_matrix_v[row - min_row, col - min_col] = v_val
    combined_matrix = np.stack((new_matrix_u, new_matrix_v), axis=-1)
    mask_uv = np.isnan(combined_matrix).any(axis=-1)
    mask_uv = mask_uv.astype(np.uint8) * 255
    new_matrix_u = np.nan_to_num(new_matrix_u, nan=0).astype(np.uint16)
    new_matrix_v = np.nan_to_num(new_matrix_v, nan=0).astype(np.uint16)
    interpolated_matrix_u = cv2.inpaint(new_matrix_u, mask_uv, inpaintRadius=3, flags=cv2.INPAINT_NS)
    interpolated_matrix_v = cv2.inpaint(new_matrix_v, mask_uv, inpaintRadius=3, flags=cv2.INPAINT_NS)

    # 创建网格进行插值
    grid_x, grid_y = np.mgrid[min_row:max_row + 1, min_col:max_col + 1]

    # 插值
    interpolated_u = griddata(unique_coords, averaged_new_u, (grid_x, grid_y), method='linear', fill_value=np.nan)
    interpolated_v = griddata(unique_coords, averaged_new_v, (grid_x, grid_y), method='linear', fill_value=np.nan)
    mask = ~np.isnan(interpolated_u) & ~np.isnan(interpolated_v)
    valid_uv = np.column_stack((interpolated_matrix_v[mask], interpolated_matrix_u[mask])).astype(int)
    valid_indices = np.argwhere(mask)
    valid_unique_coords = valid_indices + np.array([min_row, min_col])
    return valid_unique_coords,valid_uv
def map_to_equirectangular(equirect_img,img,count_img, K,D, R2, T2, output_width, output_height,img_idx, all_transforms,pixel_masks,dim2):
    h, w = img.shape[:2]
    dist_coeffs=np.array(D)
    # 创建网格坐标
    u = np.arange(w).astype(np.float32)
    v = np.arange(h).astype(np.float32)
    u, v = np.meshgrid(u, v)
    offset = np.array([[0, 0, 0]]).reshape(3, 1)
    d=0.052
    ni=1
    nr=1.33
    # 将图像坐标中心化
    uvpoints = np.stack((u.flatten(), v.flatten(), np.ones_like(u.flatten())), axis=-1)

    undistorted_points = undistort_points(uvpoints, K, dist_coeffs,width,height)
    undistorted_points = undistorted_points.reshape(-1, 2).T
    #去除不对的点
    valid_mask_0 = (0 <= undistorted_points[0, :]) & (undistorted_points[0, :] < dim2[0]) & \
                 (0 <= undistorted_points[1, :]) & (undistorted_points[1, :] < dim2[1])
    undistorted_points=undistorted_points.T[valid_mask_0].T
    new_u=u.flatten()[valid_mask_0]
    new_v=v.flatten()[valid_mask_0]

    # 计算光线方向
    ray_directions = pixel_to_ray(undistorted_points, K)
    points_glass = calculate_point_intersection(ray_directions, offset, d).squeeze()
    normal_vectors = calculate_normal_line(points_glass)
    refraction_vectors = calculate_refraction_ray(ray_directions, normal_vectors, ni, nr)


    T2_expanded=T2
    T2_expanded[2]=T2[2]+200
    T2_expanded = T2_expanded[:, np.newaxis]  /1000# 变为 (3, 1)
    # scale=5-np.linalg.norm(T2_expanded)
    scale1=d-np.linalg.norm(offset)
    scale2=5-d-np.linalg.norm(T2_expanded)
    # ray_vectors=ray_directions*scale
    ray_vectors=scale1*ray_directions+scale2*refraction_vectors
    # 旋转光线方向
    # world_directions = R2@ ray_vectors+T2_expanded
    world_directions = R2@ ray_directions
    # 归一化
    world_directions /= np.linalg.norm(world_directions, axis=0, ord=2)

    # 计算 theta 和 phi
    theta =-np.arctan2(world_directions[2, :], world_directions[0, :])  # 水平角度
    phi = np.arcsin(world_directions[1, :])  # 垂直角度
    # 映射到等距矩形坐标
    x_eq = np.round(((theta + np.pi) / (2 * np.pi) * output_width)).astype(int)
    y_eq = np.round(((phi + np.pi/2) / np.pi * output_height)).astype(int)
    # 使用布尔索引进行赋值
    valid_mask = (0 <= x_eq) & (x_eq < output_width) & (0 <= y_eq) & (y_eq < output_height)

    valid_x_eq = x_eq[valid_mask]
    valid_y_eq = y_eq[valid_mask]
    valid_new_u = new_u[valid_mask].astype(int)
    valid_new_v = new_v[valid_mask].astype(int)

    coords = np.array(list(zip(valid_y_eq, valid_x_eq)))

    # 找到唯一的 (y, x) 坐标及其索引
    unique_coords, indices = np.unique(coords, axis=0, return_inverse=True)
    # 创建用于存储总和和计数的数组
    sum_u = np.zeros(unique_coords.shape[0])
    sum_v = np.zeros(unique_coords.shape[0])
    count = np.zeros(unique_coords.shape[0])

    # 使用 np.add.at 累加总和和计数
    np.add.at(sum_u, indices, valid_new_u)
    np.add.at(sum_v, indices, valid_new_v)
    np.add.at(count, indices, 1)

    # 计算平均值
    averaged_new_u =  np.round(sum_u / count).astype(int)
    averaged_new_v =  np.round(sum_v / count).astype(int)

    valid_unique_coords,valid_uv=interpolate_points_from_img(unique_coords,averaged_new_u,averaged_new_v)
    rows, cols = valid_unique_coords[:, 0], valid_unique_coords[:, 1]
    all_transforms[img_idx][rows, cols] = valid_uv
    # all_transforms[img_idx][unique_coords[:,0], unique_coords[:,1]] = np.column_stack((averaged_new_v, averaged_new_u))
    pixel_masks[f'{img_idx:02d}']=unique_coords
    # # 只保留不超过一次的像素
    # count_mask = count_img[valid_y_eq, valid_x_eq] < 2
    # final_mask = valid_mask[valid_mask] & count_mask
    # equirect_img[unique_coords[:,0], unique_coords[:,1]] += img[averaged_new_v, averaged_new_u]
    # count_img[unique_coords[:,0], unique_coords[:,1]] += 1
    return all_transforms,pixel_masks
    # return equirect_img,count_img


def quaternion_to_rotation_matrix(q):
    """将四元数转换为旋转矩阵"""
    w, x, y, z = q
    R = np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x ** 2 + y ** 2)]
    ])
    return R
def undistort(img_path,out_path,K,D,is_fisheye,k0,dim2,dim3,scale=0.6,imshow=False):#scale=1,图像越小
    # DIM=[1200,1200]
    scale=1
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
        # undistorted_img =cv2.fisheye.undistortImage(img,K,D,Knew=k0)
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=1)
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
        undistorted_img = cv2.remap(
            img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT
        )

        # undistorted_img = cv2.fisheye.undistortImage(img, K, D, Knew=scaled_K)
    else:
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, dim1, 1, dim1)
        mapx, mapy = cv2.initUndistortRectifyMap(K, D, None, newcameramtx, dim1, 5)
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    # undistorted_img= cv2.undistort(img, K, D, None,K)
    if imshow:
        cv2.imshow("undistorted", undistorted_img)
    cv2.imwrite(out_path, undistorted_img)
    return undistorted_img,new_K

def read_camera_matrices(file_path):
    camera_matrices = {}

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for i, line in enumerate(lines):
        line = line.strip()
        # 跳过注释行
        if line.startswith('#') or not line:
            continue

        # 解析每一行
        parts = list(map(float, line.split()))

        # 提取参数
        fu, u0, v0, ar, s = parts[0:5]
        D = parts[5:9]
        quaternion = parts[9:13]  # 四元数
        translation = parts[13:16]  # 位移
        width, height = int(parts[16]), int(parts[17])

        # 转换四元数为旋转矩阵
        R = quaternion_to_rotation_matrix(quaternion)
        T = np.array(translation)

        # 保存相机的 R 和 T 矩阵
        camera_matrices[f'Camera{i-1}'] = {
            'K':np.array([[fu, 0, u0],
                       [0, fu, v0],
                       [0, 0, 1]]),
            'R': R,
            'T': T,
            'D':D,
        }

    return camera_matrices


def fill_black_areas(equirect_img):
    # Ensure the image is in the correct format (8-bit)
    if equirect_img.dtype != np.uint8:
        equirect_img = cv2.convertScaleAbs(equirect_img)  # Convert to 8-bit

    # Create a mask where the image is black (0)
    mask = (equirect_img == 0).astype(np.uint8)  # Create a binary mask
    mask = mask [:,:,0]* 255  # Convert to 255 for inpainting

    # Perform inpainting
    filled_img = cv2.inpaint(equirect_img, mask, inpaintRadius=5, flags=cv2.INPAINT_TELEA)

    return filled_img


def update_pixel_map_with_closest_image(pixel_map, pixel_masks,all_transforms, output_width, output_height,dim):
    # 定义图像中心
    img_height, img_width = dim2 # 假设每幅图像的尺寸为 (900, 1800)
    img_centers = np.array([[img_width // 2, img_height // 2]] * 6)  # 6个图像的中心


    # 初始化距离数组
    distances = np.full((output_height, output_width, 6), np.inf)  # 创建一个 (output_height, output_width, 6) 的距离数组

    # 遍历每幅图像，计算距离
    for img_idx in range(len(all_transforms)):
        img_coords = all_transforms[img_idx]
        mask_points = pixel_masks[f"{img_idx:02d}"]
        # 获取有效的坐标
        # 计算距离
        img_center_x, img_center_y = img_centers[img_idx]

        # 只计算有效坐标的距离
        if len(mask_points) > 0:
            valid_img_x = img_coords[mask_points[:, 0], mask_points[:, 1], 1]
            valid_img_y = img_coords[mask_points[:, 0], mask_points[:, 1], 0]

            # 计算有效坐标的距离
            distances[mask_points[:, 0], mask_points[:, 1], img_idx] = np.sqrt(
                (valid_img_x - img_center_x) ** 2 + (valid_img_y - img_center_y) ** 2)

        # 找到最小距离的图像索引
    closest_img_indices = np.argmin(distances, axis=2)
    inf_mask = np.all(distances == np.inf, axis=2)

    # 将这些像素的索引设置为 -1
    closest_img_indices[inf_mask] = -1
    # 更新 pixel_map
    pixel_map[:] = closest_img_indices

    return pixel_map


def extract_pixels_to_equirect(equirect_img, pixel_map, all_transforms, images,dim2):
    # 假设 images 是一个包含 6 张图像的列表，每张图像的大小为 (900, 1800, 3)
    output_height, output_width = equirect_img.shape[:2]

    # 初始化计数图像
    count_img = np.zeros((output_height, output_width), dtype=int)

    # 遍历每个图像索引
    for img_idx in range(len(all_transforms)):
        # 获取当前图像对应的像素位置
        mask = pixel_map == img_idx  # 创建掩码，找出当前图像索引的位置

        # 获取有效的 (y, x) 坐标
        valid_yx = np.argwhere(mask)  # 获取当前图像索引的所有有效 (y, x) 坐标

        # 获取对应的坐标
        img_coords = all_transforms[img_idx][valid_yx[:, 0], valid_yx[:, 1]]  # shape (N, 2)
        img_u, img_v = img_coords[:, 1].astype(int), img_coords[:, 0].astype(int)  # (u, v) --> (x, y)

        # 检查坐标是否在有效范围内
        valid_mask = (0 <= img_u) & (img_u < dim2[1]) & (0 <= img_v) & (img_v < dim2[0])

        # 从对应的图像中提取有效像素值
        if np.any(valid_mask):  # 如果有有效坐标、
            #test on line
            # image_shape = images[img_idx].shape  # 获取图像的形状
            # line_width = 10  # 可以根据需要调整
            # # 获取图像的中间位置
            # mid_y = image_shape[0] // 2  # 行数的一半（高度）
            # mid_x = image_shape[1] // 2  # 列数的一半（宽度）
            # # 创建一个全黑的图像
            # new_image = np.zeros(image_shape, dtype=np.uint8)
            # # 在中间绘制白色线条
            # # new_image[mid_y - line_width // 2: mid_y + line_width // 2, :] = 255  # 横线
            # new_image[:, mid_x - line_width // 2: mid_x + line_width // 2] = 255  # 竖线
            #
            # equirect_img[valid_yx[valid_mask][:, 0], valid_yx[valid_mask][:, 1]] += new_image[
            #     img_v[valid_mask], img_u[valid_mask]]

            equirect_img[valid_yx[valid_mask][:, 0], valid_yx[valid_mask][:, 1]] += images[img_idx][
                img_v[valid_mask], img_u[valid_mask]]


            count_img[valid_yx[valid_mask][:, 0], valid_yx[valid_mask][:, 1]] += 1  # 更新计数图像

    # 处理计数图像，避免除以零
    valid_mask_final = count_img > 0
    # equirect_img[valid_mask_final] /= count_img[valid_mask_final][:, np.newaxis]  # 归一化

    return equirect_img


annots = np.load(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco_360_1\extri_annots.npy",allow_pickle=True)

file_path = r'C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco_360_1\camera_params.txt'
camera_matrices = read_camera_matrices(file_path)
# 示例使用


# 图像合成
width, height = 2000,1000 # 等距柱状图像的大小
dim2=[1648, 1648]
equirect_img = np.zeros((height, width, 3), dtype=np.float64)
print(len(camera_matrices))
R1,T1=camera_matrices[f'Camera{0}']['R'],camera_matrices[f'Camera{0}']['T']
count_img = np.zeros((height, width), dtype=np.uint8)
# 初始化 pixel_map
pixel_map = np.full((height, width), -1, dtype=int)  # -1 表示没有使用的像素
images=[]
pixel_masks={}
# 初始化 all_transforms
all_transforms = [np.zeros((height, width, 2), dtype=int) for _ in range(6)]  # 6张图片的变换
for i in range(len(camera_matrices)):
    if i>0:
        j=6-i
    else:
        j=i
        # continue
    params=camera_matrices[f'Camera{i}']
    img_name=f'frame000000_cam{j:03d}.png'
    img_path=os.path.join(r'C:\Users\Mayn\Desktop\simulation\results\original_earth5m_0',img_name)
    out_path=os.path.join(r'C:\Users\Mayn\Desktop\simulation\results\undistorted_earth5m_0',img_name)
    os.makedirs(r'C:\Users\Mayn\Desktop\simulation\results\undistorted_earth5m_0',exist_ok=True)
    # os.makedirs(out_path,exist_ok=True)
    img = cv2.imread(img_path)
    # 读取相机图像  C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco_360_1\undistorted
    K = params['K']
    R = params['R']
    T = params['T']
    D = params['D']
    undistorted_img, new_K = undistort(img_path, out_path, K, np.array(D), True, k0=None, dim2=dim2,
                                       dim3=dim2)  # dim2=[2448,2048],dim3=[ 2448,2048]
    cv2.imwrite(out_path, undistorted_img)
    images.append(undistorted_img)
    all_transforms,pixel_masks = map_to_equirectangular(equirect_img,undistorted_img,count_img, new_K,D, R,T,width, height,i, all_transforms,pixel_masks,dim2)
    # equirect_img,count_img=map_to_equirectangular(equirect_img,undistorted_img,count_img, new_K,D, R,T,width, height,i, all_transforms)
pixel_map = update_pixel_map_with_closest_image(pixel_map,pixel_masks, all_transforms, width, height,dim2)

#保存pixel_map
color_map = np.array([
    [0, 0, 0],      # 黑色 (-1)
    [255, 255, 255], # 白色 (0)
    [255, 0, 0],     # 红色 (1)
    [0, 255, 0],     # 绿色 (2)
    [0, 0, 255],     # 蓝色 (3)
    [255, 255, 0],   # 黄色 (4)
    [0, 255, 255]    # 青色 (5)
], dtype=np.uint8)

# 使用 NumPy 的高级索引快速生成图像
image = color_map[pixel_map + 1]  # +1 是为了将 -1 映射到索引 0

# 保存图像
cv2.imwrite('pixel_map_visualization.png', image)


# valid_pixels = count_img > 0  # 创建掩码，选择有效像素
equirect_img = extract_pixels_to_equirect(equirect_img, pixel_map, all_transforms, images,dim2)
equirect_img_float = equirect_img.astype(np.float64)
# count_img_expanded = count_img[:, :, np.newaxis]
# 计算平均值
# equirect_img_float[valid_pixels] /= count_img_expanded[valid_pixels]
#
# # 处理可能的除以零情况
# equirect_img_float[count_img == 0] = 0  # 对未被赋值的像素设置为 0 或其他默认值
# 插值去除黑边
# equirect_img_float=fill_black_areas(equirect_img_float)
# 将结果转换回 uint8 类型
equirect_img = np.clip(equirect_img_float, 0, 255).astype(np.uint8)
# 保存或显示结果
cv2.imwrite('equirectangular_projection.jpg', equirect_img)
cv2.imshow('Equirectangular Projection', equirect_img)
cv2.waitKey(0)
cv2.destroyAllWindows()