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

class MyCompetitionClass
{
public:
  explicit MyCompetitionClass(ros::NodeHandle & node)
  : arm_1_has_been_zeroed_(false)
  {
    // %Tag(ADV_CMD)%
    arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
      "ariac/kitting/kitting_arm_controller/command", 10, true);

    // %EndTag(ADV_CMD)%
  }

  // %Tag(CB_CLASS)%
  /// Called when a new JointState message is received.
  void arm_1_joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
  {
    ROS_INFO_STREAM_THROTTLE(10,
      "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
    // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
    arm_1_current_joint_states_ = *joint_state_msg;
    if (!arm_1_has_been_zeroed_) {
      arm_1_has_been_zeroed_ = true;
      ROS_INFO("Sending arm to zero joint positions...");
      send_arm_to_zero_state();
    }
  }

//   void arm_2_joint_state_callback(
//     const sensor_msgs::JointState::ConstPtr & joint_state_msg)
//   {
//     ROS_INFO_STREAM_THROTTLE(10,
//       "Joint States arm 2 (throttled to 0.1 Hz):\n" << *joint_state_msg);
//     // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
//     arm_2_current_joint_states_ = *joint_state_msg;
//     if (!arm_2_has_been_zeroed_) {
//       arm_2_has_been_zeroed_ = true;
//       ROS_INFO("Sending arm 2 to zero joint positions...");
//       send_arm_to_zero_state(arm_2_joint_trajectory_publisher_);
//     }
//   }
  // %EndTag(CB_CLASS)%

  // %Tag(ARM_ZERO)%
  /// Create a JointTrajectory with all positions set to zero, and command the arm.
  void send_arm_to_zero_state() {
    // Create a message to send.
    trajectory_msgs::JointTrajectory msg;

    // Fill the names of the joints to be controlled.
    // Note that the vacuum_gripper_joint is not controllable.
    msg.joint_names.clear();
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.joint_names.push_back("linear_arm_actuator_joint");
    // Create one point in the trajectory.
    msg.points.resize(2);
    // Resize the vector to the same length as the joint names.
    // Values are initialized to 0.
    msg.points[0].positions.resize(msg.joint_names.size(), 0.0);
    // How long to take getting to the point (floating point seconds).
    msg.points[0].time_from_start = ros::Duration(0.1);

    ROS_INFO_STREAM("Sending command:\n" << msg);

    msg.points[1].positions.resize(msg.joint_names.size(), 0.0);
    msg.points[1].positions[2] = -1;
    msg.points[1].time_from_start = ros::Duration(2);
    arm_1_joint_trajectory_publisher_.publish(msg);

  }
  // %EndTag(ARM_ZERO)%


private:
  std::string competition_state_;
  ros::Publisher arm_1_joint_trajectory_publisher_;
  sensor_msgs::JointState arm_1_current_joint_states_;
  bool arm_1_has_been_zeroed_;
};