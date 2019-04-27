#!/usr/bin/env python  
import math
from numpy import *
from numpy.linalg import inv

import roslib
import rospy
import tf
import geometry_msgs.msg


# Constants
cam_frame_id = 'camera_raw'
tag_list = ['tag_0', 'tag_1', 'tag_6', 'tag_7']

def estimate_state(listener, X, P):
    # predict()
    for tag in tag_list:
        try:
            (trans,rot) = listener.lookupTransform(cam_frame_id, 
                            tag, rospy.Time(0))
            print(trans)
            print(rot)
            # Z = format_measurement(trans, rot)
            # update()

        except (tf.LookupException, tf.ConnectivityException,   
                    tf.ExtrapolationException):
            continue
        

def update(X, P, Z, H, W):
    S = H.dot(P.dot(transpose(H))) + W
    K = P.dot(transpose(H).dot(inv(S)))
    innov = Z - H.dot(X)

    X = X + K.dot(innov)
    P = P + K.dot(H.dot(P))
    return X, P


def prediction(X, P, u, A, B, V):
    X = A.dot(X) + B.dot(u)
    P = A.dot(P.dot(transpose(A))) + V
    return X, P


def main():
    rospy.init_node('pose_kf_estimator')

    listener = tf.TransformListener()

    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    # State and Covariance
    P = identity(7)
    X = array([0 for i in range(7)])

    # Error Covariances
    V, W = identity(7), identity(7)  # Assume nothing about covariance matrix


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        estimate_state(listener, X, P)

        # angular = 4 * math.atan2(trans[1], trans[0])
        # linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        # cmd = geometry_msgs.msg.Twist()
        # cmd.linear.x = linear
        # cmd.angular.z = angular
        # turtle_vel.publish(cmd)

        rate.sleep()

if __name__ == '__main__':
    main()