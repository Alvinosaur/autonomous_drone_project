#!/usr/bin/env python
import numpy as np
import cv2
import os
import argparse
import yaml
import pickle

from glob import glob
from undistort import get_undistorted

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate camera using a video of a chessboard or a sequence of images.')
    parser.add_argument('input', help='input video file or glob mask')
    parser.add_argument('out', help='output calibration yaml file')
    parser.add_argument('--debug-dir', help='path to directory where images with detected chessboard will be written',
                        default=None)
    parser.add_argument('-c', '--corners', help='output corners file', default=None)
    parser.add_argument('-fs', '--framestep', help='use every nth frame in the video', default=20, type=int)
    # parser.add_argument('--figure', help='saved visualization name', default=None)
    args = parser.parse_args()
    source = cv2.VideoCapture(2)
    # square_size = float(args.get('--square_size', 1.0))

    pattern_size = (9, 6)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    # pattern_points *= square_size
    frame_width = int(source.get(3))
    frame_height = int(source.get(4))
    out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'),
            10, (frame_width,frame_height))

    obj_points = []
    img_points = []
    h, w = 0, 0
    i = -1
    j = 0
    while (j < 50):
        i += 1
        if isinstance(source, list):
            # glob
            if i == len(source):
                break
            img = cv2.imread(source[i])
        else:
            # cv2.VideoCapture
            retval, img = source.read()
            if not retval:
                break
            if i % args.framestep != 0:
                continue

        print('Searching for chessboard in frame ' + str(i) + '...'),
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size, cv2.CALIB_CB_FILTER_QUADS)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)
        if args.debug_dir:
            img_chess = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(img_chess, pattern_size, corners, found)
            # cv2.imwrite(os.path.join(args.debug_dir, '%04d.png' % i), img_chess)
            out.write(img_chess)
        if not found:
            print 'not found'
            continue
        img_points.append(corners.reshape(1, -1, 2))
        obj_points.append(pattern_points.reshape(1, -1, 3))
        j += 1
        print 'ok'

    print('\nPerforming calibration...')
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None)
    print "RMS:", rms
    print "camera matrix:\n", camera_matrix
    print "distortion coefficients: ", dist_coefs.ravel()

    print('displaying results...')
    while (True):
        ret, frame = source.read()
        orig = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Original", orig)
        undist = get_undistorted(orig, camera_matrix, dist_coefs)
        cv2.imshow("Undistorted", undist)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # # fisheye calibration
    # rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.fisheye.calibrate(
    #     obj_points, img_points,
    #     (w, h), camera_matrix, np.array([0., 0., 0., 0.]),
    #     None, None,
    #     cv2.fisheye.CALIB_USE_INTRINSIC_GUESS, (3, 1, 1e-6))
    # print "RMS:", rms
    # print "camera matrix:\n", camera_matrix
    # print "distortion coefficients: ", dist_coefs.ravel()

    calibration = {'rms': rms, 'camera_matrix': camera_matrix.tolist(), 'dist_coefs': dist_coefs.tolist() }
    with open(args.out, 'w') as fw:
        yaml.dump(calibration, fw)
