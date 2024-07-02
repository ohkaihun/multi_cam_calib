import numpy as np
import cv2
import os
from FileUtils import save_json, read_json, getFileList, makeDir
from ChessBoard import detect_chessboard
import argparse

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

#得到单个相机的内参K和畸变系数D后，对数据去畸变
def undistort(img_path,K,D,k0,dim2,dim3,scale=0.6,imshow=False):#scale=1,图像越小
    DIM=[1200,1200]
    # balance=0.6
    if k0 is None:
        k0=K
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K=K.copy()
    scaled_K[0][0] = K[0][0] * scale  # The values of K is to scale with image dimension.
    scaled_K[1][1] = K[1][1] * scale  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0
    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    # new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), k0, dim3, cv2.CV_16SC2)##opencv中的balance难调，我们选择的是K,D到K的映射
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)



    if imshow:
        cv2.imshow("undistorted", undistorted_img)
    return undistorted_img,k0

#rootiPath expected  “/IntriImage”
def calibIntri(rootiPath,outPath, pattern, gridSize, ext, num,is_charu,is_fisheye,num_board):
    """
    pattern:Number of squares horizontally & vertically ,[4,6]
    gridSize:Square side length (in m)
    ext:Data extension ,'.png'
    num:Number of pics for calibraion
    is_charu:Charucoboard detection
    is_fisheye:Fisheye cam system or pinhole cam
    num_board:Number of boards

    calibrate intri parameters
    """
    annots = {'cams': {
    }}
    iFolder = getFileList(rootiPath, ext)
    camIds = [cam['camId'] for cam in iFolder]
    makeDir(outPath, camIds)
    # detect chessboard corners and save the results
    for cam in iFolder:
        dataList = []
        intriPara = {}
        for imgName in cam['imageList']:
            if detect_chessboard(os.path.join(rootiPath, cam['camId'], imgName), outPath, "Intri", pattern, gridSize, ext,is_charu,num_board):
                pointData = read_json(os.path.join(outPath, "PointData", "Intri", cam['camId'], imgName.replace(ext,'.json')))
                reProjErr = float(reProjection(pointData,is_charu,num_board))
                pointData['reProjErr'] = reProjErr
                dataList.append(pointData)
        num = len(dataList)# - 5
        print("cam:",cam['camId'],"use:%02d images"%num)
        if len(dataList) < num :
            print("cam:",cam['camId'], "calibrate intri failed: doesn't have enough valid image")
            continue
        dataList = sorted(dataList, key = lambda x: x['reProjErr'])
        i = 0
        for data in dataList:
            if i < num :
                data['selected'] = True
                i = i + 1
            save_json(os.path.join(outPath, "PointData", "Intri", cam["camId"], data['imageName'].replace(ext, ".json")), data)
        point2dList=[]
        point3dList=[]
        for data in dataList:
            if data['selected'] == True & is_charu :
                for board_index in range(num_board):
                    if f'mask_{board_index}'  in data:
                        point2dList.append(np.expand_dims(np.array(data[f'keyPoints2d_{board_index}'],dtype=np.float32),0))
                        point3dList.append(np.expand_dims(np.array(data[f'keyPoints3d_{board_index}'],dtype=np.float32),0))
                    else:
                        pass
            elif not(is_charu)& data['selected'] == True:
                point2dList.append(np.expand_dims(np.array(data['keyPoints2d'], dtype=np.float32), 0))
                point3dList.append(np.expand_dims(np.array(data['keyPoints3d'], dtype=np.float32), 0))
            else:
                pass

        #do intri calibration using opencv2  cv2.calibrateCamera or  cv2.fisheye.calibrate
        if not is_fisheye:
            # 采用np.stack 对矩阵进行叠加,类型必须为float32，float64不可以
            ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(point3dList, point2dList, (dataList[0]['iWidth'],dataList[0]['iHeight']), None, None,flags=cv2.CALIB_FIX_K3)
        else:
            # point3dList=np.expand_dims(np.asarray(point3dList),1)
            # point2dList=np.expand_dims(np.asarray(point2dList),1)
            ret, K, dist, rvecs, tvecs = cv2.fisheye.calibrate(point3dList, point2dList,
                                                             (dataList[0]['iWidth'], dataList[0]['iHeight']), None,
                                                             None,flags = cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 1e-6))



        #Undistort an image using the calibrated intrinsic parameters and distortion coefficients
        for imgName in cam['imageList']:
            img_path=os.path.join(os.path.join(rootiPath, cam['camId'], imgName))
            ##test using the same k to ditstort
            # if cam['camId']=='00':
            #     undistorted_img, new_k= undistort(img_path, K, dist, k0=None,dim2=[1200, 1200], dim3=[1200, 1200])
            #     k0=new_k
            # else:
            undistorted_img,new_k=undistort(img_path,K,dist,k0=None,dim2=[1200,1200],dim3=[1200,1200])
            out_path=os.path.join(os.path.join(outPath,'undistorted',cam['camId'], imgName))
            cv2.imwrite(out_path,undistorted_img)

        #save results of calibation
        annots['cams'][cam['camId']] = {
            'K': K,
            'D': dist,#np.zeros((5,1))#
            'new_K':new_k
        }
        intriPara['K'] = K.tolist()
        intriPara['dist'] = dist.tolist()#np.zeros((5,1)).tolist()#
        intriPara['new_K'] = new_k.tolist()
        save_json(os.path.join(outPath, cam['camId'] + '.json'), intriPara)
    np.save(os.path.join(outPath,'annots.npy'), annots)
    #去畸变后的图片识别角点
    #detect undistorted pics again and save pointdata
    for cam in iFolder:
        dataList = []
        intriPara = {}
        for imgName in cam['imageList']:
            if detect_chessboard(os.path.join(outPath,'undistorted', cam['camId'], imgName), outPath, "Intri_undistorted", pattern, gridSize, ext,is_charu,num_board):
                pointData = read_json(os.path.join(outPath, "PointData", "Intri_undistorted", cam['camId'], imgName.replace(ext,'.json')))
                reProjErr = float(reProjection(pointData,is_charu,num_board))
                pointData['reProjErr'] = reProjErr
                dataList.append(pointData)
        num = len(dataList)  # - 5
        print("cam:", cam['camId'], "use:%02d images" % num)
        if len(dataList) < num:
            print("cam:", cam['camId'], "calibrate intri failed: doesn't have enough valid image")
            continue
        dataList = sorted(dataList, key=lambda x: x['reProjErr'])
        i = 0
        for data in dataList:
            if i < num:
                data['selected'] = True
                i = i + 1
            save_json(os.path.join(outPath, "PointData", "Intri_undistorted", cam["camId"], data['imageName'].replace(ext, ".json")), data)

if __name__ == '__main__':
    # torch.set_default_tensor_type('torch.cuda.FloatTensor')

    parser = argparse.ArgumentParser()
    parser.add_argument('--root_path', type=str, default='../sfm1/archive/bimage_fisheye_multicharuco_360')
    parser.add_argument('--out_path', type=str, default='../sfm1/archive/output_bimage_fisheye_multicharuco_360')
    parser.add_argument('--pattern', type=int, nargs='+', default=[4, 6])
    parser.add_argument('--gridsize', type=float,default=0.197)
    parser.add_argument('--ext', type=str,default='.png')
    parser.add_argument('--num',type=int, default=18)
    parser.add_argument('--is_charu', default=False, action="store_true")
    parser.add_argument('--is_fisheye', default=False, action="store_true")
    parser.add_argument('--num_board', type=int, default=6)
    args = parser.parse_args()
    calibIntri(args.root_path, args.out_path, args.pattern,args.gridsize,args.ext,args.num,args.is_charu,args.is_fisheye,args.num_board)

###test command
#calibIntri(os.path.join("Image3", "chessboard"), "OutPut3", (9,6), 1, ".jpg", 5)
# calibIntri(os.path.join("D:/acq_data4", "chessboard"), "D:/acq_data4/OutPut", (9,6), 0.1, ".jpg", 20)
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\image_chessboard", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_chessboard", (9,6), 0.024, ".jpg", 20,is_charu=False,is_fisheye=False)
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\image", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output", (6,8), 0.024, ".jpg", 20,is_charu=True,is_fisheye=False)##11*10


##blender-fisheye-onecharucoboard
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\bimage_fisheye_charuco", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_charuco", (6,8), 0.15, ".png", 20,is_charu=True,is_fisheye=True,num_board=1)##11*10

##blender-fisheye-onechessboard
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\bimage_fisheye_chess", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_chessboard", (6,9), 0.024, ".png", 24,is_charu=False,is_fisheye=True,num_board=1)

##network-normal-multicharuboard
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\muticharucoboard", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_muticharucoboard", (4,6), 0.024, ".png", 1,is_charu=True,is_fisheye=True,num_board=8)

# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\bimage_fisheye_multicharuco_360", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco_360", (4,6), 0.197, ".png", 18,is_charu=True,is_fisheye=True,num_board=6)
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\bimage_fisheye_multicharuco", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_bimage_fisheye_multicharuco", (4,6), 0.024, ".png", 24,is_charu=True,is_fisheye=True,num_board=6)

# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\image_fisheye_chessboard", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output_fisheye_chessboard", (6,9), 0.024, ".jpg", 25,is_charu=False,is_fisheye=True)
# calibIntri(r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\image", r"C:\Users\Mayn\work\calibration\LED\sfm1\archive\output", (6,8), 0.024, ".jpg", 20,is_charu=True,is_fisheye=False)##11*10
# pointData = read_json("OutPut//PointData//Intri//0//image_1.json")
#reProjection(pointData)
