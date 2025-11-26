import numpy as np
import cv2 as cv
import glob
import os

# cor = (8, 6)
cor_w = 8
cor_h = 6
# img_path = '/home/wise/Documents/sjy3/ChessboardImages'
img_path = '/home/wise/Downloads/ChessboardImages'

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((cor_h*cor_w,3), np.float32)
objp[:,:2] = np.mgrid[0:cor_w,0:cor_h].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# images = glob.glob(os.path.join(img_path, '*.jpg'))
images = glob.glob(os.path.join(img_path, '*.png'))

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (cor_w,cor_h), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        # cv.drawChessboardCorners(img, (cor_w,cor_h), corners2, ret)
        # cv.imshow('img', img)
        # cv.waitKey(500)

cv.destroyAllWindows()
h, w = gray.shape[:2]
size = (w, h)
objpoints_for_fisheye = np.array(objpoints, dtype=np.float32).reshape(len(objpoints), 1, -1, 3)


# pinhole
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)
# cv.imwrite(os.path.join(".", "undist_pinhole2.jpg"), und_p)
Kopt, _ = cv.getOptimalNewCameraMatrix(mtx, dist, size, 0.0, size)
und_p = cv.undistort(img, mtx, dist, None, Kopt)
cv.imwrite("undist_pinhole.jpg", und_p)

# fisheye
f0 = 0.8*max(w,h)
K0 = np.array( [[f0, 0, w/2],
                [0 , f0, h/2],
                [0 , 0, 1]],np.float64)
D0 = np.zeros((4,1), np.float64)

Kf = np.eye(3, dtype=np.float64); 
Df = np.zeros((4,1), np.float64)
flags = (cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC | 
         cv.fisheye.CALIB_CHECK_COND | 
         cv.fisheye.CALIB_FIX_SKEW |
         cv.fisheye.CALIB_FIX_PRINCIPAL_POINT
         )
ret_f, Kf, Df, rf, tf = cv.fisheye.calibrate(
    objpoints_for_fisheye, imgpoints, gray.shape[::-1], K0, D0, None, None,
                                                 flags=flags,
                                                 criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,50,1e-6))
Knew = Kf.copy()
cv.fisheye.estimateNewCameraMatrixForUndistortRectify(Kf, Df, size, np.eye(3), Knew, 0.0, size, 1.0)
und_f = cv.fisheye.undistortImage(img, Kf, Df, Knew=Knew, new_size=size)
cv.imwrite("undist_fisheye.jpg", und_f)

# fs = cv.FileStorage('camera_intrinsic_250912.yaml', cv.FILE_STORAGE_WRITE)
# fs.write("camera_matrix", mtx)
# fs.write("dist_coeff", dist)
# fs.release()


print("pinhole\n")
print("카메라 내부 파라미터(3x3):\n", mtx)
print("왜곡 계수:\n", dist)
# print("camera_intrinsic.yaml 파일에 저장되었습니다.")
print("fisheye\n")
print("카메라 내부 파라미터(3x3):\n", Kf)
print("왜곡 계수:\n", Df)
# print("camera_intrinsic.yaml 파일에 저장되었습니다.")