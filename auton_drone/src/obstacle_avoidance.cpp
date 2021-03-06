#include <math.h>
#include <fstream>
#include <vector>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

using namespace std;

#define CM_TO_M (float)1/100
#define DEG_TO_RAD (float)180/pi

mavros_msgs::State current_state;
ifstream usb_stream;  // connection with arduino
usb_stream.open("dev/ttyUSB0");  // ACM- or USB-


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

vector read_ultrasonics() {
  vector<float> sensor_data;
  string raw_data_stream;
  usb_stream.get(raw_data_stream);  // char or char* ?
  sstream ss(raw_data_stream);
  while (ss.good()) {
    string substr;
    getline(ss, substr, ',');
    float val = stof(substr);
    sensor_data.pushback(val);
  }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber range_sub = nh.subscribe<sensor_msgs::Range>
            ("mavros/range", 10, range_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ROS_INFO("Waiting for FCU Connection");
        ros::spinOnce();
        rate.sleep();
    }

    uint32_t count = 0;
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;
    pose.header.frame_id = 1;

    //send a few setpoints before starting
    ROS_INFO("Sending initial points before starting offboard");
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            } else {
                ROS_INFO("Failed to set OFFBOARD");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                } else {
                    ROS_INFO("Failed to arm drone");
                }
                last_request = ros::Time::now();
            }
        }
        pose.header.seq = count;
        pose.header.stamp = ros::Time::now();
        local_pos_pub.publish(pose);
        count++;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
