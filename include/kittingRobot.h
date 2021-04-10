#include <iostream>
#include <algorithm>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>



class kittingRobot{
  public:
    explicit kittingRobot(ros::NodeHandlePtr& nh) : _nh(nh) {
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      
    }





  private:
    ros::NodeHandlePtr _nh;
    const std::string planning_group_name = "kitting_arm";
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;


};
