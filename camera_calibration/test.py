#!/usr/bin/env python
import numpy as np
import cv2
import os
import yaml
import pickle
from glob import glob

def run_main():
    source = cv2.VideoCapture(1)

    pattern_size = (9, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    # pattern_points *= square_size

    obj_points = []
    img_points = []
    h, w = 0, 0
    i = 0
    while (i < 1500):
        i += 1
        if i % 20 != 0:
            continue
        retval, img = source.read()
        if not retval:
            break

        print('Searching for chessboard in frame ' + str(i) + '...'),
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size, cv2.CALIB_CB_FILTER_QUADS)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
        # img_chess = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # cv2.drawChessboardCorners(img_chess, pattern_size, corners, found)
        if not found:
            print 'not found'
            continue
        img_points.append(corners.reshape(1, -1, 2))
        obj_points.append(pattern_points.reshape(1, -1, 3))
        print 'ok'

# load corners
#    with open('corners.pkl', 'rb') as fr:
#        img_points = pickle.load(fr)
#        obj_points = pickle.load(fr)
#        w, h = pickle.load(fr)

    print('\nPerforming calibration...')
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    print "RMS:", rms
    print "camera matrix:\n", camera_matrix
    print "distortion coefficients: ", dist_coefs.ravel()

    # # fisheye calibration
    # rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.fisheye.calibrate(
    #     obj_points, img_points,
    #     (w, h), camera_matrix, np.array([0., 0., 0., 0.]),
    #     None, None,
    #     cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, (3, 1, 1e-6))
    # print "RMS:", rms
    # print "camera matrix:\n", camera_matrix
    # print "distortion coefficients: ", dist_coefs.ravel()

    while (True):
        ret, frame = source.read()
        orig = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Original", orig)
        undist = get_undistorted(orig, camera_matrix, dist_coefs)
        cv2.imshow("Undistorted", undist)
        if cv2.waitkey(1) & 0xFF == ord('q'):
            break

    calibration = {'rms': rms, 'camera_matrix': camera_matrix.tolist(), 'dist_coefs': dist_coefs.tolist() }
    with open('calibration2.yaml', 'w') as fw:
        yaml.dump(calibration, fw)

def get_undistorted(img, cam_mtrx, dist_coefs):
    h, w = img.shape[:2]
    new_cam_mtrx, roi = cv2.getOptimalNewCameraMatrix(cam_mtrx, dist_coefs,
            (w,h), 1, (w,h))
    undistorted_img = cv2.undistort(img, cam_mtrx, dist_coefs, None,
            new_cam_mtrx)
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]
    return undistorted_img

run_main()
