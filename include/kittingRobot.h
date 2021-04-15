#include <iostream>
#include <algorithm>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <tf/tf.h>

enum collisionAction{
    ADD, REMOVE
};

// rostopic echo /ariac/kittinmove_group/feedback

geometry_msgs::Vector3 rpyFromQuat(geometry_msgs::Quaternion quat){
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    geometry_msgs::Vector3 v;
    m.getRPY(v.x, v.y, v.z);
    return v;
}

geometry_msgs::Quaternion quatFromRPY(double roll, double pitch, double yaw){
    tf::Quaternion quat;
    geometry_msgs::Quaternion q;
    quat.setRPY(roll, pitch, yaw);
    q.x = quat.x(); q.y = quat.y(); q.z = quat.z(); q.w = quat.w();
    return q;
}



void log_pose(geometry_msgs::PoseStamped p){
    std::cout << "Position : x = " << p.pose.position.x << ", y = " << p.pose.position.y << ", z = " << p.pose.position.z << std::endl;
    geometry_msgs::Vector3 v = rpyFromQuat(p.pose.orientation);
    std::cout << "Orientation : roll = " << v.x << ", pitch = " << v.y << ", yaw = " << v.z << std::endl;
}

class kittingRobot{
  public:
    explicit kittingRobot(ros::NodeHandlePtr& nh) : _nh(nh) {
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(planning_group_name);
      // advertisePickAndPlaceAction();
      std::cout << "End effector link is : " << move_group_interface->getEndEffectorLink() << std::endl;
      std::cout << "Press something to move to Home configuration" << std::endl;
      std::cin.get();
      // std::cout << "Starting pick and place" << std::endl;
      addInitialCollisionObjects();
      std::cout << "Moving to home position!" << std::endl;
      if (!moveToHomeConfig(true))
        ROS_ERROR_STREAM("Failed to move to home config!");
      
      geometry_msgs::Pose target_pose1;
      target_pose1.orientation = quatFromRPY(0, 0, 0);
      target_pose1.position.x = -0.5;
      target_pose1.position.y = 1.175;
      target_pose1.position.z = 1.25;
      std::cout << "Moving to manual pose" << std::endl;
      std::cout << "Success / fail : " << (moveToPose(target_pose1, true)) << std::endl;

      
      // std::cout << "Press after some time" << std::endl;
      // std::cin.get();
      // std::cout << "Current pose of end effector is : " << std::endl;
      // geometry_msgs::PoseStamped tar_pose = move_group_interface->getCurrentPose();
      // std::cout << "Header frame : " << tar_pose.header.frame_id << std::endl;
      // log_pose(tar_pose);
      // std::cout << "Enter new pose Z increment to plan to!" << std::endl;
      // double increment;
      // std::cin >> increment;
      

      while (_nh->ok()){
        continue;
      }
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
    moveit::core::RobotStatePtr current_state;
    std::vector<double> home_config = {0, -1.25, 1.75, 4.2, -1.5, 0};
    void advertisePickAndPlaceAction();
    void addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    bool addInitialCollisionObjects();
    void addPartialCollisionObject(std::string ID, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    bool publishToJointPositions(std::vector<double>, bool blocking = false);
    bool moveToPose(geometry_msgs::Pose target_pose, bool blocking = false);
    bool moveToJointPositions(std::vector<double>, bool blocking = false);
    bool moveToHomeConfig(bool blocking = false);

};

// linear_arm_actuator_joint, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint


bool kittingRobot::moveToHomeConfig(bool blocking) {
    current_state = move_group_interface->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    std::vector<double> config;
    config.push_back(joint_group_positions[0]);
    config.insert(config.end(), home_config.begin(), home_config.end());
    bool success = moveToJointPositions(config, blocking);
    return success;
}


bool kittingRobot::moveToPose(geometry_msgs::Pose target_pose, bool blocking){
    move_group_interface->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success and blocking)
      move_group_interface->move();
    else if (success and !blocking)
      move_group_interface->asyncMove();
    return success;
}

bool kittingRobot::moveToJointPositions(std::vector<double> j_values, bool blocking /*= false*/){
    move_group_interface->setJointValueTarget(j_values);
    bool success;
    if (!blocking){
      success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Asyncmove() called. Success = " << success << std::endl;
    }
    else
      success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

bool kittingRobot::addInitialCollisionObjects(){
    std::string id;
    std::vector<double> dimensions;
    geometry_msgs::Pose collision_pose;
    dimensions.resize(3);
    // Adding conveyor collisions
    {
      id = "conveyor_belt";
      dimensions[0] = 0.425;
      dimensions[1] = 8.95;
      dimensions[2] = 0.272;
      collision_pose.orientation.w = 1;
      collision_pose.position.x = -0.573076;
      collision_pose.position.y = 0;
      collision_pose.position.z = 0.725314;
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }
    {
      id = "conveyor_slab_front";
      dimensions[0] = 0.106;
      dimensions[1] = 9;
      dimensions[2] = 0.399;
      collision_pose.orientation.w = 1;
      collision_pose.position.x = -0.274596;
      collision_pose.position.y = 0;
      collision_pose.position.z = 0.71847;
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }
    {
      id = "conveyor_slab_back";
      dimensions[0] = 0.106;
      dimensions[1] = 9;
      dimensions[2] = 0.399;
      collision_pose.orientation.w = 1;
      collision_pose.position.x = -0.838685;
      collision_pose.position.y = 0;
      collision_pose.position.z = 0.71847;
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }
    {
      id = "linear_actuator_collision";
      dimensions[0] = 0.2;
      dimensions[1] = 10;
      dimensions[2] = 0.1;
      collision_pose.orientation.w = 1;
      collision_pose.position.x = -1.3;
      collision_pose.position.y = 0;
      collision_pose.position.z = 0.948;
      addPartialCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }

    std::cout << "Finished adding the objects" << std::endl;

}

void kittingRobot::addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = Id;
    shape_msgs::SolidPrimitive primitive;
    if (Dimensions.size()==3){
      primitive.type = primitive.BOX;
      primitive.dimensions = Dimensions;
    }
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(CollisionPose);
    if (Action == collisionAction::ADD)
      collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.push_back(collision_object);
    std::cout << "Adding " << Id << " into the world" << std::endl;
    planning_scene_interface.applyCollisionObjects(collisionObjects);


}

bool kittingRobot::publishToJointPositions(std::vector<double> joint_positions, bool blocking /*= false*/){
    
}

void kittingRobot::addPartialCollisionObject(std::string ID, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action){
    addPlanningSceneCollisionObject(ID, Dimensions, CollisionPose, Action);
    // Need to work on this later on
    /*
    std::vector<std::string> name_list;
    ros::Time start_time = ros::Time::now();
    bool not_found;
    while ((not_found = (std::find(name_list.begin(), name_list.end(), ID) == name_list.end())) 
      && ros::Duration(5) > (ros::Time::now() - start_time)){
      name_list = planning_scene_interface.getKnownObjectNames();
      std::cout << "Waiting for object " << ID << " to be added to the planning scene" << std::endl;
    }
    if (not_found){
      std::cout << "Object wasn't added to the planning scene even after 5 secs" << std::endl;
      return;
    }
    ros::Publisher planning_scene_diff_pub = _nh->advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::ServiceClient planning_client = _nh->serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    // std::cout << "LOL " << ros::service::waitForService("/ariac/kitting/get_planning_scene", ros::Duration(10)) << std::endl;
    
    if(!planning_client.call(srv)){
      ROS_WARN("Failed to call service /ariac/kitting/get_planning_scene");
    }
    else{
      std::cout << "Current scene! " << std::endl;
      moveit_msgs::PlanningScene currentScene, newScene;
      currentScene = srv.response.scene;
      moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;
      ROS_ERROR_STREAM("size of acm_entry_names before " << currentACM.entry_names.size());
      ROS_ERROR_STREAM("size of acm_entry_values before " << currentACM.entry_values.size());
      ROS_ERROR_STREAM("size of acm_entry_values[0].entries before " << currentACM.entry_values[0].enabled.size());
      currentACM.entry_names.push_back(ID);
      moveit_msgs::AllowedCollisionEntry entry;
      entry.enabled.resize(currentACM.entry_names.size(),true);
      entry.enabled[0] = false; // base_link can collide
      // row update
      currentACM.entry_values.push_back(entry);
      // std::cout << "ACM names are : ";
      for (std::size_t i = 0; i<currentACM.entry_values.size(); i++){
        // std::cout << currentACM.entry_names[i] << ", ";
        // column update
        if (currentACM.entry_names[i] == "base_link")
          currentACM.entry_values[i].enabled.push_back(false);
        else
          currentACM.entry_values[i].enabled.push_back(true);
      }
      newScene.is_diff = true;
      newScene.allowed_collision_matrix = currentACM;
      planning_scene_diff_pub.publish(newScene);
      
    }

    if (!planning_client.call(srv)){
      ROS_WARN("Failed to call service /ariac/kitting/get_planning_scene");
    }
    else{
      ROS_INFO_STREAM("Modified scene!");

      moveit_msgs::PlanningScene currentScene = srv.response.scene;

      moveit_msgs::AllowedCollisionMatrix currentACM = currentScene.allowed_collision_matrix;


      ROS_ERROR_STREAM("size of acm_entry_names after " << currentACM.entry_names.size());

      ROS_ERROR_STREAM("size of acm_entry_values after " << currentACM.entry_values.size());

      ROS_ERROR_STREAM("size of acm_entry_values[0].entries after " << currentACM.entry_values[0].enabled.size());
    }
    // std::cout << "All known names in planning scene are : ";
    // for (auto it : name_list){
    //   std::cout << it << ", ";
    // }
    // std::cout << std::endl; */

}