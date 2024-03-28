import cv2
import calibrate

img = cv2.imread('calibrate.jpg')
objpoints, imgpoints, img_corners = calibrate.find_corners(img)

intrin, dist = calibrate.solve_intrinsics(objpoints, imgpoints, img, 'fisheye')
img_undistorted = calibrate.undistort_image(intrin, dist, img, 'fisheye')

cv2.imwrite('/output/corners.jpg', img_corners)
cv2.imwrite('/output/undistorted.jpg', img_undistorted)
