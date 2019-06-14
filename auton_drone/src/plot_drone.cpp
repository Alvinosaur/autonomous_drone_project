#include <stdlib.h>
#include <vector>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "auton_drone/TagDetection.h"

#define TIME_THRESH 0.5  // 0.5 seconds max duration for considering transform

void print_transform(tf::Transform trans);


/* Creates vector of all possible detected tags, each represented with the
 * TagDetection class.
 */
void define_tags(std::vector<TagDetection> &tag_init_data){
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

/*
 * Takes average calculated pose from all detected tags' transformations
 */
tf::Vector3 take_avg(const tf::TransformListener &tl,
           std::vector<TagDetection> &tag_poses,
           int &found_tag) {
  size_t i;
  unsigned int num_detected_tags = 0;
  tf::Vector3 cam_position;  // automatically initialized to all 0's

  for (i = 0; i < tag_poses.size(); i++) {
  // loop through all tags, fuse in camera pose estimate if tag detected
    if (tag_poses[i].get_transform(tl)) {
      num_detected_tags++;
      // r_g_cam = r_g_tag + r_cam_tag^-1, assuming no rotation
      cam_position += (tag_poses[i].global_pose *
                       tag_poses[i].transform.inverse()).getOrigin();
    }
  }
  found_tag = (num_detected_tags > 0);
  // attempt to take average
  return (num_detected_tags != 0) ? cam_position/num_detected_tags : cam_position;
}

void print_transform(tf::Transform trans) {
  tf::Quaternion q = trans.getRotation();
  tf::Vector3 axis = q.getAxis();
  tf::Vector3 origin = trans.getOrigin();
  float angle = q.getAngle();
  ROS_INFO("x, y, z, xr, yr, zr, w: %f, %f, %f, %f, %f, %f, %f",
           origin.getX(), origin.getY(), origin.getZ(),
           axis.getX(), axis.getY(), axis.getZ(), angle);
}

void init_marker_list(visualization_msgs::Marker &points) {
  points.header.frame_id = "world";
  points.header.stamp = ros::Time::now();
  points.ns = "points";
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::SPHERE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.025;
  points.scale.y = 0.025;
  points.scale.z = 0.025;

  // Points are green
  points.color.r = 1.0f;
  points.color.a = 1.0;
}

tf::Vector3 get_cam_pose(const tf::TransformListener &tl,
                         std::vector<TagDetection> &tag_poses,
                         int &found_tag){
  return take_avg(tl, tag_poses, found_tag);
}

void vec_to_point(tf::Vector3 vec, geometry_msgs::Point &new_point) {
  new_point.x = vec.getX();
  new_point.y = vec.getY();
  new_point.z = vec.getZ();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_drone_pose");
  ros::NodeHandle nh;
  // ros::Subscriber tf_subscriber = nh.subscribe("/tf", 10, &tf_cb);
  ros::Rate rate(10);

  // Initialize point-trajectory visualization
  ros::Publisher marker_pub = (
    nh.advertise<visualization_msgs::Marker>("ball_trajectory", 10));
  visualization_msgs::Marker points;
  init_marker_list(points);
  geometry_msgs::Point new_point;

  // Initialize publiher to PX4 Flight Controller
  ros::Publisher px4_pose_pub = (
    nh.advertise<geometry_msgs::PoseStamped>("vision_pose/pose", 10));
  // ros::Publisher px4_pose_pub = (
  //   nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vision_pose/pose_cov", 10));
  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.header.frame_id = "world";  // NOTE: May not be necessary

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
        tf::StampedTransform(cam_pose_g, cur_time, "world", "camera_avg"));

      // Visualize point-trajectory
      vec_to_point(cam_pose, new_point);
      points.points.push_back(new_point);
      marker_pub.publish(points);
      // for point in points.points, ROS_INFO(point)

      // Publish to PX4 Flight Controller
      stamped_pose.header.stamp = cur_time;
      stamped_pose.pose.position = new_point;
      // stamped_pose.pose.orientation;  // TODO: SHOULDN'T BE 0, FIX
      px4_pose_pub.publish(stamped_pose);
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
