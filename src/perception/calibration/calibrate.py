import cv2
import numpy as np

def find_corners(img):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    objpoints = []
    imgpoints = []

    corner_criteria = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    pattern = (6, 9)

    print(pattern)

    # downsize image
    # img = cv2.resize(img, (np.shape(img)[0]//4, np.shape(img)[1]//4))

    ret, corners = cv2.findChessboardCorners(img, pattern, corner_criteria)

    if ret:
        # points corresponding to grid used
        objp = np.zeros((pattern[0] * pattern[1],3), np.float32)
        objp[:,:2] = np.mgrid[0:pattern[1],0:pattern[0]].T.reshape(-1,2)
        objp = np.expand_dims(np.asarray(objp), -2)

        objpoints.append(objp)

        corners2d = cv2.cornerSubPix(img, corners, (11,11), (-1,1), subpix_criteria)
        imgpoints.append(corners2d)

        cv2.drawChessboardCorners(img, pattern, corners2d, ret)

    else:
        print("no chessboard corners found")

    # should publish intrinsics & undistorted image
    return objpoints, imgpoints, img

def solve_intrinsics(objpoints, imgpoints, img, cam_type):
    N_OK = len(objpoints)
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]

    print(objpoints, imgpoints)

    if cam_type == 'fisheye':
        rms, _, _, _, _ = \
            cv2.fisheye.calibrate(
                np.array(objpoints, dtype=np.float64), np.array(imgpoints, dtype=np.float64),
                img.shape[:2],
                K, D, rvecs, tvecs,
                cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )
    elif cam_type == 'pinhole':
        pass
    
    return K, D

def undistort_image(K, D, img, cam_type):
    if cam_type == 'fisheye':
        # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, img.shape[:2], cv2.CV_16SC2)
        # img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR)

        print(K)
        print(D)

        img = cv2.fisheye.undistortImage(distorted=img, K=K, D=D, Knew=K)
    else:
        pass

    return img
