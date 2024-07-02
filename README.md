# multi_cam_calib
this is repository for calibration of multi-fisheye cam system, you can use charucoboard/chessboard to calibrate.
## Installation
---
## Cloning the Repository
```shell
# SSH
git clone git@github.com:/ohkaihun/multicam_calibration.git --recursive
```
or
```shell
# HTTPS
git clone https://github.com/ohkaihun/multicam_calibration --recursive
```
## Conda Environment
```
conda create -n calib python=3.8
conda activate calib
pip install  scipy g2o-python opencv-contrib-python=4.9.0.80 matplotlib 
```
## Date preparation 
Our repository expect the following dataset structure in the source path location :
```
<location>
|---root_path
    |---00
    |   |---<image 0>
    |   |---<image 1>
    |   |---...
    |---01
    |---...
|---out_path
```
## Try to run 
run calibration in 2 steps. 
- pattern: Number of squares horizontally & vertically ,[4,6]
- gridSize: Square side length (in m)
- ext: Data extension ,'.png'
- num: Number of pics for calibraion
- is_charu: Charucoboard detection
- is_fisheye: Fisheye cam system or pinhole cam
- num_board: Number of boards
```shell
python CalibIntri_fisheye.py --root_path ../sfm1/archive/bimage_fisheye_multicharuco_360  --out_path ../sfm1/archive/output_bimage_fisheye_multicharuco_360 --pattern (4,6) --gridsize 0.197 --ext .png  --num 18 --is_charu --is_fisheye  --num_board 6
python CalibExtri_fisheye.py --root_path ../sfm1/archive/output_bimage_fisheye_multicharuco_360 --pattern (4,6) --num_board 6 --num 18 --pattern (4,6)
```
then use script to visualize the results.
```shell
python pose_visualize.py --dirpath /path/to/your/outpath
```
