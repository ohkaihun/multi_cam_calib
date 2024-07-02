import os
import json

def read_json(path):
    with open(path, 'r') as f:
        data = json.load(f)
    return data

def save_json(file, data):
    if file is None:
        return 0
    if not os.path.exists(os.path.dirname(file)):
        os.makedirs(os.path.dirname(file))
    with open(file, 'w') as f:
        json.dump(data, f, indent=4)

'''
读取目录下的图像
'''
def getFileList(root, ext='.jpg'):
    rootFolder = []
    dirs = sorted(os.listdir(root))
    for dir in dirs:
        if os.path.isdir(os.path.join(root, dir)):
            dic = {}
            dic['camId'] = dir
            dic['imageList'] = [img for img in os.listdir(os.path.join(root, dir)) if img.endswith(ext)]
            dic['imageNum'] = len(dic['imageList'])
            rootFolder.append(dic)
    return rootFolder

#创建输出目录
def makeDir(outPath, cams):
    # 使用os.makedirs，父目录不存在时，会创建父目录
    for cam in cams:
        # if not os.path.lexists(os.path.join(outPath, "DrawCorner", "Extri", cam)):
        #     os.makedirs(os.path.join(outPath, "DrawCorner", "Extri", cam))
        if not os.path.lexists(os.path.join(outPath, "DrawCorner", "Intri", cam)):
            os.makedirs(os.path.join(outPath, "DrawCorner", "Intri", cam))
        # if not os.path.lexists(os.path.join(outPath, "PointData", "Extri", cam)):
        #     os.makedirs(os.path.join(outPath, "PointData", "Extri", cam))

        if not os.path.lexists(os.path.join(outPath, "undistorted", cam)):
            os.makedirs(os.path.join(outPath, "undistorted", cam))

        if not os.path.lexists(os.path.join(outPath, "PointData",'Intri_undistorted', cam)):
            os.makedirs(os.path.join(outPath, "PointData",'Intri_undistorted', cam))
        if not os.path.lexists(os.path.join(outPath, "DrawCorner",'Intri_undistorted', cam)):
            os.makedirs(os.path.join(outPath, "DrawCorner",'Intri_undistorted', cam))
        if not os.path.lexists(os.path.join(outPath, "PointData", "Intri", cam)):
            os.makedirs(os.path.join(outPath, "PointData", "Intri", cam))
    '''
    if not os.path.lexists(outPath):
        os.mkdir(outPath)
    if not os.path.lexists(os.path.join(outPath, "DrawCorner")):
        os.mkdir(os.path.join(outPath, "DrawCorner"))
    if not os.path.lexists(os.path.join(outPath, "DrawCorner", "Extri")):
        os.mkdir()
    if not os.path.lexists(os.path.join(outPath, "PointData")):
        os.mkdir(os.path.join(outPath, "PointData"))
    '''

#test = getFileList(os.path.join("Image","IntriImage"),".jpg")
#cams = [cam['camId'] for cam in test]
# makeDir("OutPut", cams)
#print(test)

