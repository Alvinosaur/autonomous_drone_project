#ifndef AUTONDRONE_TAGDETECTION_H
#define AUTONDRONE_TAGDETECTION_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#define TIME_THRESH 0.5  // 0.5 seconds max duration for considering transform

class TagDetection {
  public:
    int detected;  // status flag
    std::string cam_id;
    std::string tag_id;
    tf::StampedTransform transform;  // transform from tag to camera if detected
    tf::Transform global_pose;  // 3D vector from global origin to tag

    TagDetection(const char*, float, float, float, tf::Quaternion);

    int get_transform(const tf::TransformListener&);
};

#endif
