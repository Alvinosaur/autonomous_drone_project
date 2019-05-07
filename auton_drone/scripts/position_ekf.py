#!/usr/bin/env python
import math
from numpy import *
from numpy.linalg import inv
import time
import matplotlib.pyplot as plt
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


def init_state(listener, X, plot_data):
    # List of tags to be found
    tags_left = [tag_name for (tag_name, tag) in tag_list]

    # Loop until we've detected each tag at least once to have correct # values
    while(len(tags_left) > 0):
        print('Searching for tag...', tags_left)
        for (tag_name, tag) in tag_list:
            if (tag_name in tags_left):
                try:
                    (trans,rot) = listener.lookupTransform(cam_frame_id,
                                tag_name, rospy.Time(0))
                    X = pose_from_meas(trans, rot, tag)

                    tags_left.remove(tag_name)
                    plot_data.add_data(tag_name, X)

                except (tf.LookupException, tf.ConnectivityException,
                        tf.ExtrapolationException):
                    continue

        time.sleep(1)
    # Just return last tag that was stored
    return append(X, [[0], [0], [0]]).reshape((STATE_DIM, 1))


def generate_H():
    # Preserve position, but measurement should not have any velocity est
    H = array([[1, 0, 0, 0, 0, 0],
               [0, 1, 0, 0, 0, 0],
               [0, 0, 1, 0, 0, 0]])
    return H


def estimate_state(listener, X, P, V, W, dt, plot_data):
    A = generate_A(dt)
    H = generate_H()

    X, P = predict(X, P, A, V)
    for (tag_name, tag) in tag_list:
        try:
            (trans, rot) = listener.lookupTransform(cam_frame_id,
                            tag_name, rospy.Time(0))
            Z = pose_from_meas(trans, rot, tag)
            X, P = update(X, P, Z, H, W)
            plot_data.add_data(tag_name, Z)

        except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
            plot_data.add_data(tag_name, None)
            continue

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
    W[0][0] = 1000
    W[1][1] = 1000
    W[2][2] = 1000

    # Plotting position to compare accuracy of filter to individual
    # measurements of tags
    time_axis = []
    fig, position_plots, velocity_plots = plotter.init_plots()
    x_plot, y_plot, z_plot = position_plots
    plot_data = plotter.init_plot_data()
    plt.axis([0, 1000, -10, 10])

    rate = rospy.Rate(10.0)
    prev_time = time.time()

    init = True
    i = 0
    while not rospy.is_shutdown():
        cur_time = time.time()
        if init:
            X = init_state(listener, X, plot_data)
            init = False
        else:
            X, P = estimate_state(listener, X, P, V, W,
                                cur_time - prev_time, plot_data)
            trans= get_transform(X)
            br.sendTransform(trans, zero_rot, rospy.Time.now(),
                            "camera", "world")

        time_axis.append(cur_time)
        plot_data.add_data('filter', X)
        # plotter.plot_new_points(position_plots, plot_data, i)
        x_plot.scatter(i, float(plot_data.x['filter'][-1][0]), c = 'r')
        y_plot.scatter(i, float(plot_data.y['filter'][-1][0]), c = 'r')
        z_plot.scatter(i, float(plot_data.z['filter'][-1][0]), c = 'r')

        x_plot.scatter(i, float(plot_data.x['tag_0'][-1][0]), c = 'g')
        y_plot.scatter(i, float(plot_data.y['tag_0'][-1][0]), c = 'g')
        z_plot.scatter(i, float(plot_data.z['tag_0'][-1][0]), c = 'g')

        plt.pause(0.05)
        i += 1

        prev_time = cur_time

        rate.sleep()


if __name__ == '__main__':
    main()
