#!/usr/bin/env python
import math
from numpy import *
from numpy.linalg import inv
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import plot_pose_live as plotter


import roslib
import rospy
import tf
import geometry_msgs.msg

STATE_DIM = 6  # Dimension of state vector, [x, y, z, vx, vy, vz]
MEAS_DIM = 3

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


# Returns global translation of camera as vector3 x, y, z
def pose_from_meas(trans, rot, tag):
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    mat = inv(dot(trans_mat, rot_mat))  # camera to tag

    trans_global_tag = tf.transformations.translation_matrix(
                        [tag.x, tag.y, tag.z])

    global_cam_pose = trans_global_tag.dot(mat)
    init_cam_pos = array([[global_cam_pose[0][3]],
                            [global_cam_pose[1][3]],
                            [global_cam_pose[2][3]]])

    return init_cam_pos


def generate_A(dt):
    A = identity(STATE_DIM)
    A[0][3] = dt
    A[1][4] = dt
    A[2][5] = dt
    return A
    # A = array([[1, 0, 0, dt, 0, 0]
    #            [0, 1, 0, 0, dt, 0],
    #            [0, 0, 1, 0, 0, dt],
    #            [0, 0, 0, 1, 0, 0],
    #            [0, 0, 0, 0, 1, 0],
    #            [0, 0, 0, 0, 0, 1],


def init_state(listener, X, plot_data, time_axis):
    # List of tags to be found
    tags_left = [tag_name for (tag_name, tag) in tag_list]

    # Loop until we've detected each tag at least once to have correct # values
    while(len(tags_left) > 0):
        print('Searching for tag...')
        time.sleep(1)
        for (tag_name, tag) in tag_list:
            if (tag_name not in tags_left):
                try:
                    (trans,rot) = listener.lookupTransform(cam_frame_id,
                                tag_name, rospy.Time(0))
                    X = pose_from_meas(trans, rot, tag)

                    tags_left.remove(tag_name)
                    plot_data.x[tag_name].append(X[0])
                    plot_data.y[tag_name].append(X[1])
                    plot_data.z[tag_name].append(X[2])

                except (tf.LookupException, tf.ConnectivityException,
                        tf.ExtrapolationException):
                    continue
    time_axis.append(time.time())
    # Just return last tag that was stored
    return append(X, [[0], [0], [0]]).reshape((STATE_DIM, 1))


def generate_H():
    # Preserve position, but measurement should not have any velocity est
    H = array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0]])
    return H


def estimate_state(listener, X, P, V, W, dt, plot_data, time_axis):
    A = generate_A(dt)
    H = generate_H()

    X, P = predict(X, P, A, V)
    for (tag_name, tag) in tag_list:
        try:
            (trans, rot) = listener.lookupTransform(cam_frame_id,
                            tag_name, rospy.Time(0))
            Z = pose_from_meas(trans, rot, tag)
            X, P = update(X, P, Z, H, W)
            plot_data.add_data

        except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
            plot_data.x[tag_name].append(X[-1])
            plot_data.y[tag_name].append(X[-1])
            plot_data.z[tag_name].append(X[-1])
            continue
    time_axis.append(time.time())
    return X, P


def update(X, P, Z, H, W):
    S = H.dot(P.dot(H.T)) + W
    K = P.dot(H.T).dot(inv(S))
    innov = Z - H.dot(X)

    X = X + K.dot(innov)
    P = P - K.dot(H.dot(P))
    return X, P


def predict(X, P, A, V):
    X = A.dot(X)
    P = A.dot(P.dot(A.T)) + V
    return X, P


def get_transform(X):
    translation = X[0:3]
    return translation


def main():
    rospy.init_node('pose_kf_estimator')

    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # turtle_vel = rospy.Publisher('', geometry_msgs.msg.Twist,queue_size=1)

    # State and Covariance
    P = identity(STATE_DIM)
    # [x, y, z, vx, vy, vz, xr, yr, zr, w]
    X = array([[0] for i in range(STATE_DIM)])
    zero_rot = [0, 0, 0, 1]

    # Error Covariances
    # Assume nothing about covariance matrix
    V, W = identity(STATE_DIM), identity(MEAS_DIM)

    # Plotting position to compare accuracy of filter to individual
    # measurements of tags
    time_axis = []
    fig, position_plots, velocity_plots = plotter.init_plots()
    plot_data = plotter.init_plot_data()
    animation.FuncAnimation(fig, plotter.animate, fargs=(time_axis, plot_data.x,
                                position_plots[0]), interval=1000)
    animation.FuncAnimation(fig, plotter.animate, fargs=(time_axis, plot_data.y,
                                position_plots[1]), interval=1000)
    animation.FuncAnimation(fig, plotter.animate, fargs=(time_axis, plot_data.z,
                                position_plots[2]), interval=1000)
    plt.show()


    rate = rospy.Rate(10.0)
    prev_time = time.time()

    init = True
    while not rospy.is_shutdown():
        if init:
            X = init_state(listener, X, plot_data, time_axis)
            init = False
        else:
            X, P = estimate_state(listener, X, P, V, W,
                                time.time() - prev_time, plot_data, time_axis)
            trans= get_transform(X)
            br.sendTransform(trans, zero_rot, rospy.Time.now(),
                            "camera", "world")

        plot_data.x['filter'].append(X[0])
        plot_data.y['filter'].append(X[1])
        plot_data.z['filter'].append(X[2])

        prev_time = time.time()

        rate.sleep()

if __name__ == '__main__':
    main()
