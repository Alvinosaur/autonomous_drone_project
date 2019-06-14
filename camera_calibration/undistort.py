import cv2

def get_undistorted(img, cam_mtrx, dist_coefs):
    h, w = img.shape[:2]
    new_cam_mtrx, roi = cv2.getOptimalNewCameraMatrix(cam_mtrx, dist_coefs,
            (w,h), 1, (w,h))
    undistorted_img = cv2.undistort(img, cam_mtrx, dist_coefs, None,
            new_cam_mtrx)
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]
    return undistorted_img
