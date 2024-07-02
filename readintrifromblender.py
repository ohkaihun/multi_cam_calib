import json
import numpy as np
import cv2
import os
from FileUtils import save_json
imgs = cv2.imread(r'C:\Users\Mayn\work\calibration\LED/sfm1/archive/bimage_fisheye_charuco/2/image_18.png', cv2.IMREAD_UNCHANGED)
with open(r'C:\Users\Mayn\work\calibration\LED/sfm1/archive/bimage_fisheye_charuco/2/transforms.json', 'r') as fp:
    file = json.load(fp)
H, W = imgs.shape[:2]
camera_angle_x = float(file['camera_angle_x'])
focal = .5 * W / np.tan(.5 * camera_angle_x)
camera_intrinsics = np.array([
    [focal, 0, 0.5 * W],
    [0, focal, 0.5 * H],
    [0, 0, 1]
])
intriPara = {}
intriPara['K'] = camera_intrinsics.tolist()
save_json(os.path.join(r'C:\Users\Mayn\work\calibration\LED/sfm1/archive/output_bimage_fisheye_charuco/true_transforms.json'), intriPara)