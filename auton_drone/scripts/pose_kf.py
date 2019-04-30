#!/usr/bin/env python
import math
from numpy import *
from numpy.linalg import inv
import time

import roslib
import rospy
import tf
import geometry_msgs.msg

STATE_DIM = 11  # Dimension of state vector
MEAS_DIM = 8

class TagDetection():
    def __init__(self, name, x, y, z):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return "TagDetection(x=%f, y=%f, z=%f)" % (self.x, self.y, self.z)


# Constants
cam_frame_id = 'camera_raw'
tag0 = TagDetection("tag_0", .0635, .1397, 0)
tag1 = TagDetection("tag_1", .2159, .1397, 0)
tag7 = TagDetection("tag_7", .0635, .0635, 0)
tag6 = TagDetection("tag_6", .2159, .0635, 0)
tag_list = [('tag_0', tag0), ('tag_1', tag1), ('tag_6', tag6), ('tag_7', tag7)]


def format_measurement(trans, rot):
    return array([trans[0], trans[1], trans[2],
                     rot[0], rot[1], rot[2], rot[3], 1])

def generate_H(tag):
    H = array([[1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1*tag.x],
               [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, -1*tag.y],
               [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1*tag.z],
               [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]])
    return H

def generate_A(dt):
    A = identity(11)
    A[0][3] = dt
    A[1][4] = dt
    A[2][5] = dt
    return A
    # A = array([[1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0]
    #            [0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0],
    #            [0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0],
    #            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
    #            [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
    #            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
    #            [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
    #            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]....])


def init_state(listener, X):
    while(True):
        print('Searching for tag...')
        for (tag_name, tag) in tag_list:
            try:
                (trans,rot) = listener.lookupTransform(cam_frame_id,
                            tag_name, rospy.Time(0))
                trans_mat = array([[1, 0, 0, tag.x],
                                   [0, 1, 0, tag.y],
                                   [0, 0, 1, tag.z]])
                trans.append(1)
                trans = array(trans)
                init_pos = trans_mat.dot(trans)
                # subtract 1 to provide space for extra 1
                X = append(init_pos,
                    [0 for i in range(STATE_DIM - len(init_pos) - 2)])
                X = append(X, [1, 1])
                return X
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                continue
        time.sleep(1)



def estimate_state(listener, X, P, V, W, dt):
    A = generate_A(dt)
    assert(A.shape == P.shape)

    X, P = predict(X, P, A, V)
    for (tag_name, tag) in tag_list:
        try:
            (trans,rot) = listener.lookupTransform(cam_frame_id,
                            tag_name, rospy.Time(0))
            Z = format_measurement(trans, rot)
            H = generate_H(tag)
            X, P = update(X, P, Z, H, W)

        except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
            continue
    return X, P


def update(X, P, Z, H, W):
    S = H.dot(P.dot(H.T)) + W
    K = P.dot(H.T).dot(inv(S))
    innov = Z - H.dot(X)

    X = X + K.dot(innov)
    P = P - K.dot(H.dot(P))
    # print(K)
    return X, P


def predict(X, P, A, V):
    X = A.dot(X)
    P = A.dot(P.dot(A.T)) + V
    return X, P


def get_transform(X):
    translation = X[0:3]
    rotation = X[6:10]
    return translation, rotation


def main():
    rospy.init_node('pose_kf_estimator')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # turtle_vel = rospy.Publisher('', geometry_msgs.msg.Twist,queue_size=1)

    # State and Covariance
    P = identity(STATE_DIM)
    X = array([0 for i in range(STATE_DIM)])  # [x, y, z, vx, vy, vz, xr, yr, zr, w, 1]

    # Error Covariances
    V, W = identity(STATE_DIM), identity(MEAS_DIM)  # Assume nothing about covariance matrix

    rate = rospy.Rate(10.0)
    prev_time = time.time()

    init = True
    while not rospy.is_shutdown():
        br.sendTransform
        if init:
            X = init_state(listener, X)
            # print('initX: ', X)
            init = False
        else:
            X, P = estimate_state(listener, X, P, V, W, time.time() - prev_time)
            # print('New: ', X)
            trans, rot = get_transform(X)
            br.sendTransform(trans, rot, rospy.Time.now(), "camera", "world")

        prev_time = time.time()


        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)
        # print("X: ", X)
        # print("P: ", P)

        rate.sleep()

if __name__ == '__main__':
    main()
