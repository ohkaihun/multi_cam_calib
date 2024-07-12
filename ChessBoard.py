import numpy as np
import cv2
import os
from FileUtils import save_json

def getChessboard3d(pattern, gridSize):
    object_points = np.zeros((pattern[1]*pattern[0], 3), np.float32)
    # 注意：这里为了让标定板z轴朝上，设定了短边是x，长边是y
    object_points[:,:2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1,2)
    object_points[:, [0, 1]] = object_points[:, [1, 0]]
    object_points = object_points * gridSize
    return object_points

def create_chessboard(pattern, gridSize,is_charu,num_board):
    # print('Create chessboard {}'.format(pattern))
    # pattern1=(pattern[0]-1,pattern[1]-1)
    """
    data format              add multi-board detection
    """
    if is_charu:
        template = {
            'imageName': None,
            'iWidth': 0,
            'iHeight': 0,
            'selected': False,
            'reProjErr': 0.0,
            'num_board_detected': 0,
        }
        for board_index in range(num_board):
            keypoints3d = getChessboard3d(pattern, gridSize=gridSize)
            keypoints2d = np.zeros((keypoints3d.shape[0], 2))
            template[f'keyPoints3d_{board_index}']=keypoints3d.tolist()
            template[f'keyPoints2d_{board_index}']=keypoints2d.tolist()
        return template
    else:
        keypoints3d = getChessboard3d(pattern, gridSize=gridSize)
        keypoints2d = np.zeros((keypoints3d.shape[0], 2))
        template = {
            'imageName': None,
            'iWidth': 0,
            'iHeight': 0,
            'selected': False,
            'reProjErr': 0.0,
            'num_board_detected': 0,
            'keyPoints3d':keypoints3d.tolist(),
            'keyPoints2d':keypoints2d.tolist(),
        }
        return template




def _findChessboardCorners(img, pattern):
    # print("basic function")
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    retval, corners = cv2.findChessboardCorners(img, pattern,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_FILTER_QUADS)
    if not retval:
        return False, None
    corners = cv2.cornerSubPix(img, corners, (11, 11), (-1, -1), criteria)
    corners = corners.squeeze()
    return True, corners


def _buildBoardCorrdination_3D(squareSize,patternsize):
    squareSize = squareSize / 100
    cornersWorldWhole=[]
    for y in range(patternsize[1] - 1):
        for x in range(patternsize[0] - 1):
            cornersWorldWhole.append([[x * squareSize, y * squareSize, 0]])

    cornersWorldWhole = np.array(cornersWorldWhole, 'float32')
    return cornersWorldWhole

def get_total_marker(SQUARES_VERTICALLY,SQUARES_HORIZONTALLY):
    y_1=SQUARES_HORIZONTALLY//2
    y_2=SQUARES_HORIZONTALLY-y_1
    x_1=SQUARES_VERTICALLY//2
    x_2=SQUARES_VERTICALLY-x_1
    return x_2*y_1+x_1*y_2

def get_total_corner(pattern):
    return pattern[0]*pattern[1]
def _findCharucoboardCorners(img, pattern,board_index):
    # define charucoboard
    ARUCO_DICT = cv2.aruco.DICT_4X4_250
    SQUARES_VERTICALLY = pattern[0]+1  # Number of squares vertically:5
    SQUARES_HORIZONTALLY = pattern[1]+1  # Number of squares horizontally:7
    SQUARE_LENGTH = 0.03    # Square side length (in pixels)    square_size_m = square_size / 1000
    MARKER_LENGTH = 0.015   # ArUco marker side length (in pixels)    marker_size_m = marker_size / 1000
    # cornersId = []
    # cornersImg = []
    # cornersWorld = []
    # cornersWorldWhole=[]
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    num_marker_total=get_total_marker(SQUARES_VERTICALLY,SQUARES_HORIZONTALLY)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary, np.arange(num_marker_total)+num_marker_total*board_index)
    # board.setLegacyPattern(True)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)
    #detect markers
    marker_corners, marker_ids, rejectedCandidates = detector.detectMarkers(img)
    distCoeffsInit = np.zeros((4, 1))
    # cameraMatrixInit = np.array([[500., 0., 600],
    #                              [0., 500., 600],
    #                              [0., 0., 1.]])



    corners, ids, rejected, _ = detector.refineDetectedMarkers(
        image=img,
        board=board,
        detectedCorners=marker_corners,
        detectedIds=marker_ids,
        rejectedCorners=rejectedCandidates,
        # cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
    )


    #refine resolution 精细分辨率
    if len(marker_corners) > 0:
        # SUB PIXEL DETECTION
        for corner in corners:
            cv2.cornerSubPix(img, corner,
                             winSize=(3, 3),
                             zeroZone=(-1, -1),
                             criteria=criteria)
    if marker_ids is not None:
        number, chCorners, chIds = cv2.aruco.interpolateCornersCharuco(corners, ids, img, board, None,
                                                                      None, None, None, 2)
    else:
        number=0
        chCorners=None
        chIds=None
    if number >= 8:#at least 8 corners detected
        return True, chCorners, chIds
    else:
        return False,chCorners, chIds
        #     cornersImg.append(np.array(chCorners))
            # corners = chCorners.squeeze()
            # chids=chIds.squeeze()
            # correspondingCornersWorld = []
            # for i in chIds[:, 0]:
            #     correspondingCornersWorld.append(cornersWorldWhole[i])
            # cornersId.append(chIds)
            # cornersWorld.append(np.array(correspondingCornersWorld))
            # image = cv.aruco.drawDetectedCornersCharuco(img, chCorners, chIds)
    # return True,chCorners,chIds
def distortBackPoints(x, y, cameraMatrix, dist):

    fx = cameraMatrix[0,0]
    fy = cameraMatrix[1,1]
    cx = cameraMatrix[0,2]
    cy = cameraMatrix[1,2]
    k1 = dist[0][0] * -1
    k2 = dist[0][1] * -1
    k3 = dist[0][2] * -1
    k4 =dist[0][3]*-1
    x = (x - cx) / fx
    y = (y - cy) / fy

    r2 = x*x + y*y

    xDistort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)
    yDistort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2)

    xDistort = xDistort * fx + cx;
    yDistort = yDistort * fy + cy;

    return xDistort, yDistort
def findChessboardCorners(img, pointData, pattern,is_charu,is_fisheye,num_board,type,old_k,dist):
    '''
    pointData:        template = {
                                'imageName': None,
                                'iWidth': 0,
                                'iHeight': 0,
                                'selected': False,
                                'reProjErr': 0.0,
                                'num_board_detected': 0,
                                'keyPoints3d_{0}':
                                ..
                                'keyPoints3d_{5}':}
    return show
    '''
    #print("findChessBoardCorner")
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    charu_flag=is_charu           #different way of generating show pic
    # Find the chessboard corners
    if not charu_flag:
        for func in [_findChessboardCorners, _findChessboardCornersAdapt]:#add charucoboard
            ret, corners = func(gray, pattern)
            if ret:
                charu_flag = False
                break
        show = img.copy()
        show = cv2.drawChessboardCorners(show, pattern, corners, ret)
        assert corners.shape[0] == len(pointData['keyPoints2d'])
        #if missing corners:   遍历 chids 中的索引值,并将对应的 points3d 元素赋值到新的列表中

        corners = corners.squeeze()
        corners = np.hstack((corners, np.ones((corners.shape[0], 1))))[:, :2]
        pointData['keyPoints2d'] = corners.tolist()
        return show
    # Find the charucoboard corners
    else:
        show = img.copy()
        # counting detected boards
        num_board_detected=0#counting detected boards
        for board_index in range(num_board):
            objp=[]
            ##return coordinates&chids of corners of board
            ret, corners, chids = _findCharucoboardCorners(gray, pattern,board_index)  ##return coordinates&chids of corners of board
            if ret:
                # compute total num of corners
                num_board_detected+=1
                num_corners_total=get_total_corner(pattern)
                # Label each corner point on the board
                show = cv2.aruco.drawDetectedCornersCharuco(show, corners, chids+num_corners_total*board_index)
                new_points3d = [0] * corners.shape[0]
                mask=[]
                for i in range(chids.shape[0]):
                    new_points3d[i] = pointData[f'keyPoints3d_{board_index}'][chids[i, 0]]
                    mask.append(chids[i,0])
                pointData[f'keyPoints3d_{board_index}'] = new_points3d #save points of No.board_index charuco-board
                mask=np.array(mask).tolist()
                pointData[f'mask_{board_index}']=mask
                if type=="Intri_backproject" and is_fisheye:

                    newcorner = corners.squeeze()
                    objp = np.column_stack([(newcorner[:, 0] - old_k[0, 2]) / old_k[0, 0],
                                                 (newcorner[:, 1] - old_k[1, 2]) / old_k[1, 1],
                                                 np.zeros(len(newcorner))])
                    objp = objp.astype(np.float32)
                    objp = np.expand_dims(objp, axis=1)
                    rvec = np.array([[[0., 0., 0.]]])
                    tvec = np.array([[[0., 0., 0.]]])
                    newcorner, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, old_k, dist)
                    # newcorner = np.hstack((newcorner, np.ones((newcorner.shape[0], 1))))[:, :2]
                    pointData[f'keyPoints2d_{board_index}'] = newcorner.tolist()
                # elif type=="Intri_undistorted" and not is_fisheye:
                #     newcorner = cv2.undistortPoints(corners, old_k, dist)
                #     pointData[f'keyPoints2d_{board_index}'] = newcorner.tolist()
                else:
                    corners = corners.squeeze()
                    corners = np.hstack((corners, np.ones((corners.shape[0], 1))))[:, :2]
                    pointData[f'keyPoints2d_{board_index}'] = corners.tolist()
        pointData['num_board_detected'] = num_board_detected
        if num_board_detected ==0:
            return None
        else:
            return show
    # found the corners
    # show = img.copy()
    # if charu_flag:
    #     show =cv2.aruco.drawDetectedCornersCharuco(show, corners, chids)
    #     new_points3d = [0] * corners.shape[0]
    #     for i in range(chids.shape[0]):
    #         new_points3d[i] = pointData['keyPoints3d'][chids[i, 0]]
    #     pointData['keyPoints3d']=new_points3d
    # else:
    #     show = cv2.drawChessboardCorners(show, pattern, corners, ret)
    # assert corners.shape[0] == len(pointData['keyPoints2d'])
    ##if missing corners:   遍历 chids 中的索引值,并将对应的 points3d 元素赋值到新的列表中

    # corners = corners.squeeze()
    # corners = np.hstack((corners, np.ones((corners.shape[0], 1))))[:, :2]
    # pointData['keyPoints2d'] = corners.tolist()
    # return show

def _findChessboardCornersAdapt(img, pattern):
    # print("Adapt mode")
    img = cv2.adaptiveThreshold(img, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY, 21, 2)
    return _findChessboardCorners(img, pattern)

#imgPath:带路径的图片名，outPath，输出目录，type ,Intri或者是Extri,ext后缀，jpg或者png
def detect_chessboard(imgPath, outPath, type, pattern, gridSize, ext,is_charu,is_fisheye,num_board,old_k,dist):
    """
    type:Original corners detection / Undistorted corners detection
    pattern:Number of squares horizontally & vertically ,[4,6]
    gridSize:Square side length (in m)
    ext:Data extension ,'.png'
    is_charu:Charucoboard detection
    num_board:Number of boards

    return Ture
    """
    pointData = create_chessboard(pattern, gridSize,is_charu,num_board) # Pointdata format
    pointData['imageName'] = imgPath.split(os.sep)[-1]
    img = cv2.imread(imgPath)
    show = findChessboardCorners(img, pointData, pattern,is_charu,is_fisheye,num_board,type,old_k,dist) #Show is a picture with blue corner points attached with id.
    pointData['iHeight'] = img.shape[0]
    pointData['iWidth'] = img.shape[1]
    if show is None:
        print(imgPath,":find chessboard corners failed")
        return False,pointData
    else:
        # save_json(os.path.join(outPath, "PointData", type,  os.sep.join(imgPath.split(os.sep)[-2:]).replace(ext, ".json")), pointData) # save pointdata
        cv2.imwrite(os.path.join(outPath, "DrawCorner", type, os.sep.join(imgPath.split(os.sep)[-1:])), show)  #save pic
        return True,pointData
