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

enum collisionAction{
    ADD, REMOVE
};

class kittingRobot{
  public:
    explicit kittingRobot(ros::NodeHandlePtr& nh) : _nh(nh) {
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(planning_group_name);
      // advertisePickAndPlaceAction();
      std::cout << "Press something to perform pick and place" << std::endl;
      std::cin.get();
      std::cout << "Starting pick and place" << std::endl;
      // visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>("linear_arm_actuator");
      /*visual_tools->prompt("Press next to print robot details!");
      ROS_INFO_NAMED("summa", "Planning frame : %s", move_group_interface->getPlanningFrame().c_str() );
      ROS_INFO_NAMED("summa", "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());
      ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
      std::copy(move_group_interface->getJointModelGroupNames().begin(), move_group_interface->getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
      visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to move robot to J positions");
      current_state = move_group_interface->getCurrentState();
      current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
      std::cout << "Joint model group names are : ";
      std::copy(joint_model_group->getActiveJointModelNames().begin(), joint_model_group->getActiveJointModelNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
      std::cout << std::endl;
      joint_group_positions[0] = -1.5;
      move_group_interface->setJointValueTarget(joint_group_positions);
      bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success)
        move_group_interface->move();
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (joint space goal) %s", success ? "" : "FAILED");
      visual_tools->prompt("Press next to exit");*/
    }





  private:
    ros::NodeHandlePtr _nh;
    const std::string planning_group_name = "kitting_arm";
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::RobotStatePtr current_state;
    std::vector<double> joint_group_positions;
    void advertisePickAndPlaceAction();
    void addPlanningSceneCollisionObject(std::string Id, std::vector<int> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    bool addInitialCollisionObjects();
    bool moveToPose(geometry_msgs::Pose target_pose);
    
};

// linear_arm_actuator_joint, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint


bool kittingRobot::addInitialCollisionObjects(){

}

void kittingRobot::addPlanningSceneCollisionObject(std::string Id, std::vector<int> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = Id;
    shape_msgs::SolidPrimitive primitive;
    if (Dimensions.size()==3){
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[0] = Dimensions[0];
      primitive.dimensions[1] = Dimensions[1];
      primitive.dimensions[2] = Dimensions[2];
    }
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(CollisionPose);
    if (Action == collisionAction::ADD)
      collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.push_back(collision_object);
    std::cout << "Adding " << Id << " into the world" << std::endl;
    planning_scene_interface.addCollisionObjects(collisionObjects);


}