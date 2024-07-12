import numpy as np

filename=r'C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco_360\extri_annots.npy'
file=np.load(filename,allow_pickle=True)
print(file)