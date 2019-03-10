#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define TIME_THRESH 2  // 2 seconds max duration for considering transform

class TagDetection {
  public:
    int *success;  // status flag
    std::string cam_id = "camera_raw";
    std::string tag_id;
    tf::StampedTransform transform;  // transform from tag to camera if detected
    tf::Transform global_pose;  // 3D vector from global origin to tag

    TagDetection(const char*, float, float, float, tf::Quaternion);

    int get_transform(const tf::TransformListener &tl){
    /* Gets most recent transform and sets status flag for whether this
     * transform is recent enough to be used. Also sets status to false if no
     * transform is present.
     */
      try {
        tl.lookupTransform(cam_id, tag_id, ros::Time(0), transform);
        *success = (abs((ros::Time::now() - transform.stamp_).toSec())
                    <= TIME_THRESH);
      } catch(tf::TransformException ex) {
        *success = 0;
      }
      ROS_INFO("%d", *success);
      return *success;
    }
};

TagDetection::TagDetection(const char *tag, float x, float y, float z,
                           tf::Quaternion rot) {
  success = (int*)malloc(sizeof(int));
  *success = 0;  // initialize to false
  tag_id.assign(tag, strlen(tag));
  global_pose.setOrigin(tf::Vector3(x, y, z));
  global_pose.setRotation(rot);
}


std::vector<TagDetection> define_tags(){
/* Creates vector of all possible detected tags, each represented with the
 * TagDetection class.
 */
  std::vector<TagDetection> tag_init_data;

  tf::Quaternion zero_rot;
  zero_rot.setRPY(0, 0, 0);

  TagDetection tag0("tag_0", 0, 0, 0, zero_rot);
  TagDetection tag1("tag_1", -0.1, 0, 0, zero_rot);
  TagDetection tag2("tag_2", 0, 0.1, 0, zero_rot);

  tag_init_data.push_back(tag0);
  tag_init_data.push_back(tag1);
  tag_init_data.push_back(tag2);

  return tag_init_data;
}

tf::Vector3 get_cam_pose(const tf::TransformListener &tl,
                         std::vector<TagDetection> tag_poses,
                         int* success){
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
  *success = (num_detected_tags > 0);
  // attempt to take average
  return (num_detected_tags != 0) ? cam_position/num_detected_tags : cam_position;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_drone_pose");
  ros::NodeHandle nh;
  // ros::Subscriber tf_subscriber = nh.subscribe("/tf", 10, &tf_cb);
  ros::Rate rate(10);
  size_t i;
  int success = 0;
  std::string global_tag_suffix("_g");

  tf::TransformBroadcaster transform_br;
  tf::TransformListener tag_listener;

  std::vector<TagDetection> tag_poses = define_tags();

  tf::Transform cam_pose_g;

  tf::Quaternion zero_rot;
  zero_rot.setRPY(0, 0, 0);
  cam_pose_g.setRotation(zero_rot);

  while (nh.ok()) {
    ros::Time cur_time = ros::Time::now();
    tf::Vector3 cam_pose = get_cam_pose(tag_listener, tag_poses, &success);
    // cam_pose_g = get_cam_pose(tag_listener, tag0_pose_g, tag1_pose_g);

    if (success) {
    // only set new camera pose if tags were detected
      cam_pose_g.setOrigin(cam_pose);
      transform_br.sendTransform(
        tf::StampedTransform(cam_pose_g, cur_time, "world", "camera"));
    }

    // constant tag position in world
    for (i = 0; i < tag_poses.size(); i++) {

      // ROS_INFO("%ld: %d", i, tag_poses[i].success);
      if (*(tag_poses[i].success)) {
        transform_br.sendTransform(
          tf::StampedTransform(tag_poses[i].global_pose, cur_time, "world",
                               tag_poses[i].tag_id + global_tag_suffix));
      }
    }

    rate.sleep();
    ros::spinOnce();
  }

}
