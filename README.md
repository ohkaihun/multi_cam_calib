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
## Data preparation 
Our repository expect the following dataset structure in the source path location :
```
<location>
|---root_path
    |---frame000000_cam000.png
    |---frame000000_cam001.png
    |---...
    |---frame000000_cam005.png
    |---frame000001_cam000.png
    |---frame000001_cam001.png
    |---...
|---out_path
```
## Try to run 
run calibration in 1 step. 
- pattern: Number of squares horizontally & vertically ,[4,6]
- gridSize: Square side length (in m)
- ext: Data extension ,'.png'
- num: Number of pics for calibraion
- is_charu: Charucoboard detection
- is_fisheye: Fisheye cam system or pinhole cam
- num_board: Number of boards
- num_cam: Number of CAMERAS
```shell
python calibrate_v1.py --root_path ../sfm1/archive/bimage_fisheye_multicharuco_360_1 --out_path ../sfm1/archive/output_bimage_fisheye_multicharuco_360_1 --pattern 4 6 --gridsize 0.1968 --ext .png --num_pic 18 --is_charu --is_fisheye --num_board 6 --num_cam 6
```
then use script to visualize the results.
```shell
python pose_visualize.py --dirpath /path/to/your/outpath
```
