#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "auton_drone/TagDetection.h"

#define TIME_THRESH 0.5  // 0.5 seconds max duration for considering transform


void define_tags(std::vector<TagDetection> &tag_init_data){
/* Creates vector of all possible detected tags, each represented with the
 * TagDetection class.
 */
  tf::Quaternion zero_rot;
  zero_rot.setRPY(0, 0, 0);

  TagDetection tag0("tag_0", .0635, .1397, 0, zero_rot);
  TagDetection tag1("tag_1", .2159, .1397, 0, zero_rot);
  TagDetection tag7("tag_7", .0635, .0635, 0, zero_rot);
  TagDetection tag6("tag_6", .2159, .0635, 0, zero_rot);

  tag_init_data.push_back(tag0);
  tag_init_data.push_back(tag1);
  tag_init_data.push_back(tag6);
  tag_init_data.push_back(tag7);
}

tf::Vector3 get_cam_pose(const tf::TransformListener &tl,
                         std::vector<TagDetection> &tag_poses,
                         int &found_tag){
  size_t i;
  unsigned int num_detected_tags = 0;
  tf::Vector3 cam_position;  // automatically initialized to all 0's

  for (i = 0; i < tag_poses.size(); i++) {
  // loop through all tags, fuse in camera pose estimate if tag detected
    if (tag_poses[i].get_transform(tl)) {
      num_detected_tags++;
      // r_g_cam = r_g_tag + r_cam_tag^-1, assuming no rotation
      cam_position += (tag_poses[i].global_pose.getOrigin() +
                       tag_poses[i].transform.inverse().getOrigin());
    }
  }
  found_tag = (num_detected_tags > 0);
  // attempt to take average
  return (num_detected_tags != 0) ? cam_position/num_detected_tags : cam_position;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_drone_pose");
  ros::NodeHandle nh;
  // ros::Subscriber tf_subscriber = nh.subscribe("/tf", 10, &tf_cb);
  ros::Rate rate(10);

  std::string global_tag_suffix("_g");
  std::vector<TagDetection> tag_poses;

  tf::TransformBroadcaster transform_br;
  tf::TransformListener tag_listener;
  tf::Transform cam_pose_g;
  tf::Quaternion zero_rot;
  zero_rot.setRPY(0, 0, 0);

  size_t i = 0;
  int found_tag = 0;

  define_tags(tag_poses);
  cam_pose_g.setRotation(zero_rot);

  while (nh.ok()) {
    ros::Time cur_time = ros::Time::now();
    tf::Vector3 cam_pose = get_cam_pose(tag_listener, tag_poses, found_tag);
    // cam_pose_g = get_cam_pose(tag_listener, tag0_pose_g, tag1_pose_g);

    if (found_tag) {
    // only set new camera pose if tags were detected
      cam_pose_g.setOrigin(cam_pose);
      transform_br.sendTransform(
        tf::StampedTransform(cam_pose_g, cur_time, "world", "camera"));
    }

    // constant tag position in world
    for (i = 0; i < tag_poses.size(); i++) {

      // ROS_INFO("%ld: %d", i, tag_poses[i].success);
      if (tag_poses[i].detected) {
        transform_br.sendTransform(
          tf::StampedTransform(tag_poses[i].global_pose, cur_time, "world",
                               tag_poses[i].tag_id + global_tag_suffix));
      }
    }

    rate.sleep();
    ros::spinOnce();
  }

}
