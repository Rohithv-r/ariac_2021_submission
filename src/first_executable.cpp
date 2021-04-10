#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Order.h>
#include <nist_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

boost::shared_ptr<const nist_gear::LogicalCameraImage> msg;
boost::shared_ptr<const nist_gear::LogicalCameraImage> msg1;

void lc_1_2_callback(const ros::TimerEvent& e) {
    msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2");
    std::cout << "Callback 1_2" << ros::Time::now().sec << std::endl;
}

void lc_3_4_callback(const ros::TimerEvent& e) {
    msg1 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4");
    std::cout << "Callback 3_4" << ros::Time::now().sec << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "subscriber_timer_trial");
    ros::NodeHandlePtr nh1(new ros::NodeHandle);
    ros::Timer lc_1_2 = nh1->createTimer(ros::Duration(3),lc_1_2_callback);
    ros::Timer lc_3_4 = nh1->createTimer(ros::Duration(3),lc_3_4_callback);
    ros::spin();
}