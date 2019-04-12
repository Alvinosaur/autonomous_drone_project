#include "auton_drone/TagDetection.h"

#define TIME_THRESH 0.5  // 0.5 seconds max duration for considering transform

/*
 * Initialize a tag with its unique identifier/name and global pose
 */
TagDetection::TagDetection(const char *tag, float x, float y, float z,
                           tf::Quaternion rot) {
  detected = 0;  // initialize to false
  tag_id.assign(tag, strlen(tag));
  cam_id.assign("camera_raw", strlen("camera_raw"));
  global_pose.setOrigin(tf::Vector3(x, y, z));
  global_pose.setRotation(rot);
}

/*
 * Gets most recent transform and sets status flag for whether this
 * transform is recent enough to be used. Also sets status to false if no
 * transform is present.
 */
int TagDetection::get_transform(const tf::TransformListener &tl){
  try {
    tl.lookupTransform(cam_id, tag_id, ros::Time(0), transform);
    detected = (abs((ros::Time::now() - transform.stamp_).toSec())
                <= TIME_THRESH);
  } catch(tf::TransformException ex) {
    detected = 0;
  }
  return detected;
}

