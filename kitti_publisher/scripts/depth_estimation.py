import cv2
import numpy as np

block_size = 9
min_disparity = 0
max_disparity = 128
num_disparity = max_disparity - min_disparity
uniqueness_ratio = 1
speckle_window_size = 200
speckle_range = 3
disp12_max_diff = 0

def estimate_depth(left_img, right_img):
    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disparity,
        numDisparities=num_disparity,
        blockSize=block_size,
        uniquenessRatio=uniqueness_ratio,
        speckleWindowSize=speckle_window_size,
        speckleRange=speckle_range,
        disp12MaxDiff=disp12_max_diff,
        P1=16 * 1 * block_size * block_size,
        P2=64 * 1 * block_size * block_size,
    )

    disparity_SGBM = stereo.compute(left_img, right_img)
    disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255, beta=0, norm_type=cv2.NORM_MINMAX)
    disparity_SGBM = np.uint8(disparity_SGBM)

    return disparity_SGBM
