import numpy as np

filename=r'C:\Users\Mayn\work\calibration\LED\sfm1\test_chessboard\annots.npy'
file=np.load(filename,allow_pickle=True)
print(file)