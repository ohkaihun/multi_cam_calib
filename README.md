# multi_cam_calib
this is repository for calibration of multi-fisheye cam system, you can use charucoboard/chessboard to calibrate.
## Installation
---
## Cloning the Repository
```shell
# SSH
git clone git@github.com:/ohkaihun/multi_cam_calib.git --recursive
```
or
```shell
# HTTPS
git clone https://github.com/ohkaihun/multi_cam_calib --recursive
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
- dict: aruco dict type
- row: vertically square num
- col：horizonally square num
- marker_size ： in mm
- square_size : in mm
- gridSize: Square side length (in mm)
- ext: Data extension ,'.png'
- num: Number of pics for calibraion
- is_charu: Charucoboard detection
- is_fisheye: Fisheye cam system or pinhole cam
- num_board: Number of boards
- num_cam: Number of CAMERAS
- root_cam :which cam as rootcam to save results. default cam0
```shell
python calibrate_v2.py --root_path ../sfm1/archive/bimage_fisheye_multicharuco_360_1 --out_path ../sfm1/archive/output_bimage_fisheye_multicharuco_360_1 --pattern 4 6 --gridsize 0.1968 --ext .png --num_pic 18 --is_charu --is_fisheye --num_board 6 --num_cam 6
python calibrate_v2.py --root_path ../LED/sfm1/archive/bimage_perspective_multicharuco_180 --out_path ../LED/sfm1/archive/output_bimage_perspective_multicharuco_180 --dict DICT_4X4_1000 --row 5 --col 5 --marker_size 12.0 --square_size 16.0 --gridsize 16 --ext .png --num_pic 40 --is_charu --num_board 25 --num_cam 8 --root_cam 2
```
then use script to visualize the results.
```shell
python pose_visualize.py --dirpath /path/to/your/outpath
```
## BA unit test 
run unit test 
```shell
python unit_test_g2o.py 
```
