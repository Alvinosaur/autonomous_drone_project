#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera_rect/image_rect", 1);
  ros::Publisher camera_info_pub = (
    nh.advertise<sensor_msgs::CameraInfo>("/camera_rect/camera_info", 10));

  cv::VideoCapture cap(1);  // video source 1 for external usb camera
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return -1;
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.frame_id = "camera";
  std_msgs::Header img_header;
  img_header.frame_id = "tag";

  cam_info_msg.height = 480;
  cam_info_msg.width = 640;
  // TODO: use getopt to pass in json file containing these intrinsics
  // cam_info_msg.D = {-1.30081323e-01, 4.47666537e-01, -1.78161543e-04,
  //                   -1.15875402e-03, -4.83188066e-01};
  // cam_info_msg.K = {521.61643826, 0.0, 329.0603408,
  //                   0.0, 520.40489422, 237.78245555,
  //                   0.0, 0.0, 1.0};
  cam_info_msg.D.push_back(-1.30081323e-01);
  cam_info_msg.D.push_back(4.47666537e-01);
  cam_info_msg.D.push_back(-1.78161543e-04);
  cam_info_msg.D.push_back(-1.15875402e-03);
  cam_info_msg.D.push_back(-4.83188066e-01);

  cam_info_msg.K[0] = 521.61643826;
  cam_info_msg.K[2] = 329.0603408;
  cam_info_msg.K[4] = 520.40489422;
  cam_info_msg.K[5] = 237.78245555;
  cam_info_msg.K[8] = 1.0;

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    ros::Time cur_time = ros::Time::now();
    cap >> frame;
    img_header.stamp = cur_time;
    cam_info_msg.header.stamp = cur_time;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = (
        cv_bridge::CvImage(img_header, "bgr8", frame).toImageMsg());

      // msg = (
      //   cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());
      pub.publish(msg);
      camera_info_pub.publish(cam_info_msg);
      cv::waitKey(1);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
