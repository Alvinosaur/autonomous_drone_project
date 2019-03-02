#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

tf::Vector3 get_cam_pose(tf::TransformListener tl){
  tf::Vector3 tag0 = tf::Vector3(0.2, 0.2, 0);
  tf::Vector3 tag1 = tf::Vector3(0.2, 0.7588, 0);

  // Need check to see if timestamp is too old since don't want to use old
  // transform if lost track of tag for a while
  tl.waitForTransform("camera", "tag_0", ros::Time(0), ros::Duration(10.0));
  tl.waitForTransform("camera", "tag_1", ros::Time(0), ros::Duration(10.0));

  tf::StampedTransform tag0_cam;
  tf::StampedTransform tag1_cam;

  try {
    tl.lookupTransform("camera", "tag_0", ros::Time(0), tag0_cam);
    tl.lookupTransform("camera", "tag_1", ros::Time(0), tag1_cam);
    tf::Vector3 tag0_cam_vec = tag0_cam.getOrigin();
    tf::Vector3 tag1_cam_vec = tag1_cam.getOrigin();

    double fused_x = (0.5*(tag0.getX() + tag0_cam_vec.x()) +
                      0.5*(tag1.getX() + tag1_cam_vec.x()));
    double fused_y = (0.5*(tag0.getY() + tag0_cam_vec.y()) +
                      0.5*(tag1.getY() + tag1_cam_vec.y()));
    double fused_z = (0.5*(tag0.getZ() + tag0_cam_vec.z()) +
                      0.5*(tag1.getZ() + tag1_cam_vec.z()));
    return tf::Vector3(fused_x, fused_y, fused_z);

  } catch(tf::TransformException ex) {
    return tf::Vector3(0,0,0);  //TODO: Implement better error handling!
  }
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
    tf::Vector3 cam_pose = get_cam_pose(tag_listener);
    cam_pose_g.setOrigin(cam_pose);

    //transform_br.sendTransform(
    //  tf::StampedTransform(cam_pose, cur_time, "world", "camera"));
    transform_br.sendTransform(
      tf::StampedTransform(tag0_pose_g, cur_time, "world", "tag_0"));
    transform_br.sendTransform(
      tf::StampedTransform(tag1_pose_g, cur_time, "world", "tag_1"));
    rate.sleep();
    ros::spinOnce();
  }

}
