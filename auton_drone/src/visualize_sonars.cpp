#include <stdlib.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "auton_drone/UltrasonicArray.h"

const float PI = 3.14159;
const int MAX_DIST = 50;
// scales of each vector that get updated by the sonar_data_cb callback below
float vec0 = 1.0, vec90 = 1.0, vec180 = 1.0, vec270 = 1.0;

float scale_distance(float raw_distance) {
  if (raw_distance > MAX_DIST) return 15;
  else return raw_distance / 2;
}

// use ConstPtr here to avoid copying entire msg, just getting ptr by pass by
// reference
void sonar_data_cb(const auton_drone::UltrasonicArray::ConstPtr& sensor_data) {
  vec0 = scale_distance(sensor_data->sonar0.dist);
  vec90 = scale_distance(sensor_data->sonar90.dist);
  vec180 = scale_distance(sensor_data->sonar180.dist);
  vec270 = scale_distance(sensor_data->sonar270.dist);
}

geometry_msgs::Pose* define_poses() {
  geometry_msgs::Pose vector0_pose, vector90_pose, vector180_pose, vector270_pose;
  vector0_pose.position.x = 0.5;
  vector0_pose.position.y = 0;
  vector0_pose.position.z = 0;
  vector0_pose.orientation.x = 1.0;
  vector0_pose.orientation.y = 0.0;
  vector0_pose.orientation.z = 0.0;
  vector0_pose.orientation.w = 1.0;

  vector90_pose.position.x = 0;
  vector90_pose.position.y = 0.5;
  vector90_pose.position.z = 0;
  vector90_pose.orientation.x = 0.0;
  vector90_pose.orientation.y = 0.0;
  vector90_pose.orientation.z = 1.0;
  vector90_pose.orientation.w = 1.0;

  vector180_pose.position.x = -0.5;
  vector180_pose.position.y = 0;
  vector180_pose.position.z = 0;
  vector180_pose.orientation.x = 0.0;
  vector180_pose.orientation.y = 0.0;
  vector180_pose.orientation.z = 1.0;
  vector180_pose.orientation.w = 0.0;

  vector270_pose.position.x = 0;
  vector270_pose.position.y = -0.5;
  vector270_pose.position.z = 0;
  vector270_pose.orientation.x = 0.0;
  vector270_pose.orientation.y = 0.0;
  vector270_pose.orientation.z = -1.0;
  vector270_pose.orientation.w = 1.0;

  geometry_msgs::Pose *all_poses = (
      (geometry_msgs::Pose*)calloc((size_t)4, sizeof(geometry_msgs::Pose)));
  all_poses[0] = vector0_pose;
  all_poses[1] = vector90_pose;
  all_poses[2] = vector180_pose;
  all_poses[3] = vector270_pose;

  return all_poses;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "sonar_vector_display");
  ros::NodeHandle n;
  ros::Rate r(3);  // publish at rate of 3Hz

  // publishes arrow markers to rviz for visualization
  ros::Publisher pub_vectors = (
      n.advertise<visualization_msgs::Marker>("visualization_marker", 10));

  // subscribes to sonar sensor output of Arduino
  ros::Subscriber sub_sonars = n.subscribe("/sensors/sonars", 10, &sonar_data_cb);

  // frame for all arrow markers
  std::string main_frame = "/main";

  // constant poses of each vector to be displayed
  geometry_msgs::Pose *all_poses = define_poses();

  // Set our initial shape type to be a cube
  uint32_t arrow_shape_id = visualization_msgs::Marker::ARROW;
  uint32_t add_shape_id = visualization_msgs::Marker::ADD;

  while (ros::ok()) {
    visualization_msgs::Marker vector0, vector90, vector180, vector270;
    // Set the frame ID and timestamp
    ros::Time cur_time = ros::Time::now();
    vector0.header.frame_id = main_frame;
    vector0.header.stamp = cur_time;
    vector90.header.frame_id = main_frame;
    vector90.header.stamp = cur_time;
    vector180.header.frame_id = main_frame;
    vector180.header.stamp = cur_time;
    vector270.header.frame_id = main_frame;
    vector270.header.stamp = cur_time;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    vector0.ns = main_frame;
    vector0.id = 0;
    vector90.ns = main_frame;
    vector90.id = 1;
    vector180.ns = main_frame;
    vector180.id = 2;
    vector270.ns = main_frame;
    vector270.id = 3;

    // set marker type to be arrow
    vector0.type = arrow_shape_id;
    vector90.type = arrow_shape_id;
    vector180.type = arrow_shape_id;
    vector270.type = arrow_shape_id;

    vector0.action = add_shape_id;
    vector90.action = add_shape_id;
    vector180.action = add_shape_id;
    vector270.action = add_shape_id;

    // Set the pose of the marker.
    vector0.pose = all_poses[0];
    vector90.pose = all_poses[1];
    vector180.pose = all_poses[2];
    vector270.pose = all_poses[3];

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    vector0.scale.x = vec0;
    vector0.scale.y = 0.5;
    vector0.scale.z = 0.5;

    vector90.scale.x = vec90;
    vector90.scale.y = 0.5;
    vector90.scale.z = 0.5;

    vector180.scale.x = vec180;
    vector180.scale.y = 0.5;
    vector180.scale.z = 0.5;

    vector270.scale.x = vec270;
    vector270.scale.y = 0.5;
    vector270.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    vector0.color.r = 1.0f;
    vector0.color.g = 0.0f;
    vector0.color.b = 0.0f;
    vector0.color.a = 0.5;

    vector90.color.r = 0.0f;
    vector90.color.g = 1.0f;
    vector90.color.b = 0.0f;
    vector90.color.a = 0.5;

    vector180.color.r = 0.0f;
    vector180.color.g = 0.0f;
    vector180.color.b = 1.0f;
    vector180.color.a = 0.5;

    vector270.color.r = .50f;
    vector270.color.g = .50f;
    vector270.color.b = 0.0f;
    vector270.color.a = 0.5;

    vector0.lifetime = ros::Duration();
    vector90.lifetime = ros::Duration();
    vector180.lifetime = ros::Duration();
    vector270.lifetime = ros::Duration();

    pub_vectors.publish(vector180);
    pub_vectors.publish(vector270);
    pub_vectors.publish(vector0);
    pub_vectors.publish(vector90);

    ros::spinOnce();
    r.sleep();
  }
}

