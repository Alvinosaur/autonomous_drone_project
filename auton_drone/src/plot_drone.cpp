#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

tf::Transform get_cam_pose(const tf::TransformListener &tl, tf::Transform &tag0, tf::Transform &tag1){
  tf::StampedTransform tag0_cam;
  tf::StampedTransform tag1_cam;
  tf::Transform cam_tag0;
  tf::Transform cam_tag1;
  tf::Vector3 cam_tag0_vec;

  // Need check to see if timestamp is too old since don't want to use old
  // transform if lost track of tag for a while
  tl.waitForTransform("camera_raw", "tag_0", ros::Time(0), ros::Duration(10.0));
  tl.waitForTransform("camera_raw", "tag_1", ros::Time(0), ros::Duration(10.0));

    tl.lookupTransform("camera_raw", "tag_0", ros::Time(0), tag0_cam);
    tl.lookupTransform("camera_raw", "tag_1", ros::Time(0), tag1_cam);
    cam_tag0 = tag0_cam.inverse();
    cam_tag1 = tag1_cam.inverse();
    // cam_tag0_vec = cam_tag0*cam_tag0.getOrigin();
    return cam_tag0;
    // tl.lookupTransform("camera", "tag_1", ros::Time(0), tag1_cam);

    // tf::Vector3 tag0_cam_vec = tag0_cam.getOrigin();
    // tf::Vector3 tag0_origin = tag0.getOrigin();
    // tf::Vector3 tag1_cam_vec = tag1_cam.getOrigin();

    // double fused_x = (tag0_origin.getX() + tag0_cam.x());
    // double fused_y = (tag0_origin.getY() + tag0_cam.y());
    // double fused_z = (tag0_origin.getZ() + tag0_cam.z());
    // return tf::Vector3(fused_x, fused_y, fused_z);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "apriltag_drone_pose");
  ros::NodeHandle nh;
  // ros::Subscriber tf_subscriber = nh.subscribe("/tf", 10, &tf_cb);
  ros::Rate rate(10);

  tf::TransformBroadcaster transform_br;
  tf::TransformListener tag_listener;

  tf::Transform tag0_pose_g;
  tf::Transform tag1_pose_g;
  tf::Transform cam_pose_g;

  tf::Quaternion zero_rot;
  zero_rot.setRPY(0, 0, 0);

  tag0_pose_g.setOrigin(tf::Vector3(0.2, 0.2, 0));
  tag0_pose_g.setRotation(zero_rot);
  tag1_pose_g.setOrigin(tf::Vector3(0.2, 0.7588, 0));  // 22in two 8x11 paper sheets
  tag1_pose_g.setRotation(zero_rot);
  cam_pose_g.setRotation(zero_rot);

  while (nh.ok()) {
    ros::Time cur_time = ros::Time::now();
    // tf::Vector3 cam_pose = get_cam_pose(tag_listener, tag0_pose_g, tag1_pose_g);
    // cam_pose_g.setOrigin(cam_pose);
    cam_pose_g = get_cam_pose(tag_listener, tag0_pose_g, tag1_pose_g);

    transform_br.sendTransform(
      tf::StampedTransform(cam_pose_g, cur_time, "tag_0_g", "camera"));
    transform_br.sendTransform(
      tf::StampedTransform(tag0_pose_g, cur_time, "world", "tag_0_g"));
    transform_br.sendTransform(
      tf::StampedTransform(tag1_pose_g, cur_time, "world", "tag_1_g"));

    rate.sleep();
    ros::spinOnce();
  }

}
