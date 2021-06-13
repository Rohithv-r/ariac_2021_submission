#include <iostream>
#include <algorithm>
#include <vector>
#include <tuple>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Time.h>
#include <ariac_2021_submission/addPlanningSceneCollision.h>
#include <ariac_2021_submission/kittingPickUpStaticObjectAction.h>
#include <ariac_2021_submission/kittingPickUpConveyorObjectAction.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/LogicalCameraImage.h>



enum collisionAction{
    ADD, REMOVE, MOVE
};

// rostopic echo /ariac/kittinmove_group/feedback

static geometry_msgs::Vector3 rpyFromQuat(geometry_msgs::Quaternion quat){
    // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Quaternion q1(quat.x, quat.y, quat.z, quat.w);

    // tf::Matrix3x3 m(q);
    tf2::Matrix3x3 m(q1);
    geometry_msgs::Vector3 v;
    m.getRPY(v.x, v.y, v.z);
    return v;
}

static geometry_msgs::Quaternion quatFromRPY(double roll, double pitch, double yaw){
    // tf::Quaternion quat;
    tf2::Quaternion quat;
    geometry_msgs::Quaternion q;
    quat.setRPY(roll, pitch, yaw);
    q.x = quat.x(); q.y = quat.y(); q.z = quat.z(); q.w = quat.w();
    return q;
}

struct RPY{
    double R, P, Y;
    RPY(geometry_msgs::Vector3 rpy){
      R = rpy.x;
      P = rpy.y;
      Y = rpy.z;
    }
    RPY(double r, double p, double y) : R(r), P(p), Y(y) { }
};

tf2::Quaternion getNetRotation(std::vector<RPY> rpy_vector){
    tf2::Quaternion net_quat, quat;
    net_quat.setRPY(0,0,0);
    for (auto i : rpy_vector){
      quat.setRPY(i.R, i.P, i.Y);
      net_quat = net_quat * quat;
    }
    return net_quat;
}

geometry_msgs::Pose transformToPose(geometry_msgs::TransformStamped t){
    geometry_msgs::Pose p;
    p.position.x = t.transform.translation.x;
    p.position.y = t.transform.translation.y;
    p.position.z = t.transform.translation.z;
    p.orientation = t.transform.rotation;
    return p;
}

bool isVectorDiffLesser(std::vector<double> v1, std::vector<double> v2, double diff=0.1){
    bool diffLesser = true;
    if (v1.size() == v2.size()){
      for (std::size_t i = 0; i < v1.size(); i++){
        diffLesser = diffLesser && (abs(v1[i]-v2[i]) <= diff);
      }
      return diff;
    }
    else
      return false;
}

/*struct moveGroupFeedbackStruct{
    int status;
    std::string state, prev_state;
};*/


void log_pose(geometry_msgs::PoseStamped p){
    std::cout << "Position : x = " << p.pose.position.x << ", y = " << p.pose.position.y << ", z = " << p.pose.position.z << std::endl;
    geometry_msgs::Vector3 v = rpyFromQuat(p.pose.orientation);
    std::cout << "Orientation : roll = " << v.x << ", pitch = " << v.y << ", yaw = " << v.z << std::endl;
}

enum errorCode{
    SUCCESS, GRIPPER_FAILED, NO_PLAN_FOUND
};

class kittingRobot{
  public:
    explicit kittingRobot(ros::NodeHandlePtr& nh) : _nh(nh) {
      tfListenerPtr = boost::make_shared<tf2_ros::TransformListener>(tfBuffer);
      jointPositionPublisher = _nh->advertise<trajectory_msgs::JointTrajectory>("kitting_arm_controller/command", 10);
      // movingFeedbackSub = _nh->subscribe("move_group/feedback", 3, &kittingRobot::movingFeedbackCallback, this);
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(planning_group_name);
      // advertisePickAndPlaceAction();
      std::cout << "End effector link is : " << move_group_interface->getEndEffectorLink() << std::endl;
      /*std::cout << "Press something to move to manual pose" << std::endl;
      std::cin.get();*/
      // std::cout << "Starting pick and place" << std::endl;
      // std::cout << "Moving to home position!" << std::endl;
      bool k;
      gantry_next_goal.position.x = gantry_next_goal.position.y = gantry_next_goal.position.z = -3;
      gantry_next_goal.orientation = quatFromRPY(0,0,0);
      gantry_next_goal_sub = _nh->subscribe<geometry_msgs::Pose>("ariac/gantry/next_goal", 3, &kittingRobot::gantryNextGoalCb, this);
      track_gantry_timer = _nh->createTimer(ros::Duration(0.025), &kittingRobot::track_gantry_timer_callback, this);

      addInitialCollisionObjects();
      advertiseActions();
      advertiseServices();

      k = moveToHomeConfig(true, true, false);
      std::cout << "Returned from function movetohome. SUCCESS = " << k << std::endl;
      std::cout << "Initializing stuff!" << std::endl;
      // std::cout << "Waiting for 2 seconds before continuing" << std::endl;
      // ros::Duration d(2);
      // d.sleep();
      /*{
        std::vector<double> cur_jvs = move_group_interface->getCurrentJointValues();
        cur_jvs[0] = 1;
        publishToJointPositions(cur_jvs, 0.2);
        std::cout << "Exited jointpositionspublish! " << std::endl;
      }*/
      // move_group_interface->stop(); 

      /*if (!moveToHomeConfig(true))
        ROS_ERROR_STREAM("Failed to move to home config!");
      else
        std::cout << "Moved to home position" << std::endl;*/
      
      /*geometry_msgs::Pose target_pose1;
      target_pose1.orientation = quatFromRPY(0, 0, 0);
      target_pose1.position.x = -0.5;
      target_pose1.position.y = 0;
      target_pose1.position.z = 1.25;
      std::cout << "Moving to manual pose" << std::endl;
      std::cout << "Success / fail : " << (moveToPose(target_pose1, true)) << std::endl;*/

      
      // std::cout << "Press after some time" << std::endl;
      // std::cin.get();
      // std::cout << "Current pose of end effector is : " << std::endl;
      // geometry_msgs::PoseStamped tar_pose = move_group_interface->getCurrentPose();
      // std::cout << "Header frame : " << tar_pose.header.frame_id << std::endl;
      // log_pose(tar_pose);
      // std::cout << "Enter new pose Z increment to plan to!" << std::endl;
      // double increment;
      // std::cin >> increment;
      
      ros::Rate rate(10);

      while (_nh->ok()){
        ros::spinOnce();
        rate.sleep();
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
    geometry_msgs::Pose gantry_next_goal;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    moveit::core::RobotStatePtr current_state;
    tf2_ros::Buffer tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    ros::ServiceServer addPlanningSceneCollisionService;
    ros::ServiceClient gripperControl;
    // moveGroupFeedbackStruct moveGroupFeedback;
    ros::Subscriber joint_states_sub;
    ros::Subscriber gantry_next_goal_sub;
    int wrist_1_joint_index;
    bool objectAttached;
    bool finishedProbe = false;
    errorCode gripperMonitoredFJTStatus = errorCode::SUCCESS;
    bool gripperMonitoredFJTFailed = false;
    ros::Timer track_gantry_timer;
    bool distanceViolated = false;
    std::string currentObjectType = "";
    void track_gantry_timer_callback(const ros::TimerEvent& e);
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_state);
    void gantryNextGoalCb(const geometry_msgs::PoseConstPtr& pose);
    void jointStatesConveyorCallback(const sensor_msgs::JointStateConstPtr& joint_state);
    // void movingFeedbackCallback(const moveit_msgs::MoveGroupActionFeedbackConstPtr& msg);
    boost::shared_ptr<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpStaticObjectAction>> pick_as;
    boost::shared_ptr<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpConveyorObjectAction>> pick_conveyor_as;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_jt_traj_client;
    // ros::ServiceServer moveToJointPositionsService;
    // ros::ServiceServer moveToJointPositionsService;
    std::vector<double> home_config = {0, -1.25, 1.75, 4.2, -1.5, 0.25};   // linear,
    // std::vector<double> pick_config = { };
    // double actualEffort;
    void fjtFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void fjtDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void fjtActiveCb();
    void fjtMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void fjtMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void fjtMonitorActiveCb();
    void advertisePickAndPlaceAction();
    void advertiseServices();
    void advertiseActions();
    bool addCollisionServiceCallback(ariac_2021_submission::addPlanningSceneCollisionRequest& req, ariac_2021_submission::addPlanningSceneCollisionResponse &res);
    bool addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    bool addInitialCollisionObjects();
    void sendEmptyTrajectoryToFJTClient();
    void sendStopTrajectoryToFJTClient();
    void sendConveyorEmptyTrajectoryToFJTClient();
    geometry_msgs::TransformStamped getTransform(std::string frame_name, std::string origin_frame = "world");
    std::tuple<double, moveit_msgs::RobotTrajectory> getCartesianPath(geometry_msgs::Pose initial_pose, geometry_msgs::Pose final_pose, bool collisionCheck=false);
    geometry_msgs::TransformStamped getCamTransform(geometry_msgs::Pose finalPose);
    geometry_msgs::Point getObjectGraspError(geometry_msgs::Pose eefPose);
    void addPartialCollisionObject(std::string ID, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    // ros::Subscriber movingFeedbackSub;
    std::tuple<bool, moveit_msgs::RobotTrajectory> getShortestOfNPaths(moveit::planning_interface::MoveGroupInterfacePtr interface, int count);
    moveit_msgs::RobotTrajectory getShortestPath(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans);
    ros::Publisher jointPositionPublisher;
    bool publishToJointPositions(std::vector<double> joint_positions, double time_from_start = 2);  // This is a blocking function!
    bool moveToPose(geometry_msgs::Pose target_pose, bool blocking = false, bool gripperMonitoring = false);
    bool moveToPoseWhileCM(geometry_msgs::Pose target_pose);
    bool moveToPoseWhileCMandGM(geometry_msgs::Pose target_pose);
    bool moveToJointPositions(std::vector<double>, bool blocking = false, bool collisionMonitoring = false);
    bool moveToJointPositionsWhileCM(std::vector<double>);
    bool moveToJointPositionsWhileCMandGM(std::vector<double> j_values);
    bool moveToHomeConfig(bool front = true, bool blocking = false, bool collisionMonitoring = false);
    bool moveTowardsObject(geometry_msgs::Pose pose, bool front, bool blocking = false, bool gripperMonitoring = false);
    bool moveTowardsConveyorObject(geometry_msgs::Pose pose, ros::Time lastPoseUpdateTime, double frontDistance = 1);
    bool moveTowardsObjectWhileMonitoring(geometry_msgs::Pose pose, bool front);
    // void probe(geometry_msgs::Vector3 direction);
    bool PickAndPlace(std::string ID, geometry_msgs::Pose initialPose, geometry_msgs::Pose finalPose, double offset = 0.15);
    bool conveyorPickAndPlace(geometry_msgs::Pose initialPose, ros::Time lastPoseUpdateTime, geometry_msgs::Pose finalPose, double objectHeight, double offset = 0.15);
    void probe(moveit_msgs::RobotTrajectory trajectory, double vel_scale = 0.1, double acc_scale = 0.1);
    void probeConveyor(moveit_msgs::RobotTrajectory trajectory, double objectHeight, geometry_msgs::Pose pose, ros::Time previousUpdateTime);
    bool compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance = 0.1);
    void staticPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpStaticObjectGoalConstPtr &goal);
    void conveyorPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpConveyorObjectGoalConstPtr &goal);
    void gripperStateCb(const nist_gear::VacuumGripperStateConstPtr& state);
    
    
};

// linear_arm_actuator_joint, shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint

void kittingRobot::advertiseActions(){
    pick_as = boost::make_shared<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpStaticObjectAction>>(*_nh, "kitting_pick_and_place_static_action", boost::bind(&kittingRobot::staticPickAndPlaceCallback, this, _1), false);
    pick_conveyor_as = boost::make_shared<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpConveyorObjectAction>>(*_nh, "kitting_pick_and_place_conveyor_action", boost::bind(&kittingRobot::conveyorPickAndPlaceCallback, this, _1), false);
    pick_as->start();
    pick_conveyor_as->start();
    follow_jt_traj_client = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("kitting_arm_controller/follow_joint_trajectory");
}

void kittingRobot::advertiseServices(){
    addPlanningSceneCollisionService = _nh->advertiseService("add_kitting_planning_scene_collision", &kittingRobot::addCollisionServiceCallback, this);
    gripperControl = _nh->serviceClient<nist_gear::VacuumGripperControl>("arm/gripper/control");
    gripperControl.waitForExistence(ros::Duration(10));
}

bool kittingRobot::addCollisionServiceCallback(ariac_2021_submission::addPlanningSceneCollisionRequest& req, 
  ariac_2021_submission::addPlanningSceneCollisionResponse &res){
    res.success = addPlanningSceneCollisionObject(req.ID, req.Dimensions, req.CollisionPose, collisionAction::ADD);
    return res.success;
}

geometry_msgs::TransformStamped kittingRobot::getTransform(std::string frame_name, std::string origin_frame){
    geometry_msgs::TransformStamped transformStamped;
    bool gotTransform = false;
    while (!gotTransform){
      try{
        transformStamped = tfBuffer.lookupTransform(origin_frame, frame_name,
                                ros::Time(0));
        gotTransform = true;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.1).sleep();
      }
    }
    return transformStamped;
}

geometry_msgs::TransformStamped kittingRobot::getCamTransform(geometry_msgs::Pose finalPose){
    geometry_msgs::TransformStamped cam_1_2_transform = getTransform("logical_camera_1_2_frame");
    geometry_msgs::TransformStamped cam_3_4_transform = getTransform("logical_camera_3_4_frame");
    double diff1 = abs(cam_1_2_transform.transform.translation.x - finalPose.position.x) + abs(cam_1_2_transform.transform.translation.x - finalPose.position.y);
    double diff2 = abs(cam_3_4_transform.transform.translation.x - finalPose.position.x) + abs(cam_3_4_transform.transform.translation.x - finalPose.position.y);
    if (diff1 < diff2)
      return cam_1_2_transform;
    else
      return cam_3_4_transform;
}

geometry_msgs::Point kittingRobot::getObjectGraspError(geometry_msgs::Pose eefPose){
    ROS_ERROR_STREAM("getObjectGraspError fn called!");
    geometry_msgs::TransformStamped cam_1_2_transform = getTransform("logical_camera_1_2_frame");
    geometry_msgs::TransformStamped cam_3_4_transform = getTransform("logical_camera_3_4_frame");
    double diff1 = abs(cam_1_2_transform.transform.translation.x - eefPose.position.x) + abs(cam_1_2_transform.transform.translation.x - eefPose.position.y);
    double diff2 = abs(cam_3_4_transform.transform.translation.x - eefPose.position.x) + abs(cam_3_4_transform.transform.translation.x - eefPose.position.y);
    // geometry_msgs::TransformStamped cam_transform;
    std::string cam_frame = "";
    nist_gear::LogicalCameraImageConstPtr msg;
    if (diff1 < diff2){
      // cam_transform = cam_1_2_transform;
      cam_frame = "logical_camera_1_2_frame";
      msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2", ros::Duration(3));
    }
    else{
      // cam_transform =  cam_3_4_transform;
      cam_frame = "logical_camera_3_4_frame";
      msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4", ros::Duration(3));
    }
    // if (msg->models.size() > 0)
    geometry_msgs::Point error;
    bool success = false;
    for (std::vector<nist_gear::Model>::const_iterator i = msg->models.begin(); i!=msg->models.end(); i++){
        if ((i->type == currentObjectType) && isVectorDiffLesser(std::vector<double>{eefPose.position.x, eefPose.position.y}, 
          std::vector<double>{i->pose.position.x, i->pose.position.y})){
            std::cout << "Yes! Found the object we were checking for grasp error!" << std::endl;
            geometry_msgs::PoseStamped p, world_pose;
            p.header.frame_id = cam_frame;
            p.pose = i->pose;
            world_pose = tfBuffer.transform(p,"world");
            // Rotate wrist 3 jt by - (world pose yaw)
            // error.x = eefPose.position.x - world_pose.pose.position.x;
            // error.y = eefPose.position.y - world_pose.pose.position.y;
            error.z = rpyFromQuat(world_pose.pose.orientation).z;
            std::cout << "Error YAW is : " << error.z << std::endl;
            std::vector<double> currentConfig = move_group_interface->getCurrentJointValues();
            currentConfig[6] += error.z;
            success = moveToJointPositions(currentConfig, true, false);
            // std::cout << "Error is : " << error.x << ", " << -error.y << std::endl;
            // return error;
        }
    }
    
    if (success){
      if (diff1 < diff2){
        // cam_transform = cam_1_2_transform;
        cam_frame = "logical_camera_1_2_frame";
        msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2", ros::Duration(3));
      }
      else{
        // cam_transform =  cam_3_4_transform;
        cam_frame = "logical_camera_3_4_frame";
        msg = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4", ros::Duration(3));
      }
      for (std::vector<nist_gear::Model>::const_iterator i = msg->models.begin(); i!=msg->models.end(); i++){
        if ((i->type == currentObjectType) && isVectorDiffLesser(std::vector<double>{eefPose.position.x, eefPose.position.y}, 
          std::vector<double>{i->pose.position.x, i->pose.position.y})){
            std::cout << "Yes! Found the object again we were checking for grasp error!" << std::endl;
            geometry_msgs::PoseStamped p, world_pose;
            p.header.frame_id = cam_frame;
            p.pose = i->pose;
            world_pose = tfBuffer.transform(p,"world");
            // Rotate wrist 3 jt by - (world pose yaw)
            error.x = eefPose.position.x - world_pose.pose.position.x;
            error.y = eefPose.position.y - world_pose.pose.position.y;
            // error.z = rpyFromQuat(world_pose.pose.orientation).z;
            std::cout << "YAW after moving to rectify orientation is : " << rpyFromQuat(world_pose.pose.orientation).z << std::endl;
            // std::vector<double> currentConfig = move_group_interface->getCurrentJointValues();
            // currentConfig[6] += error.z;
            // success = moveToJointPositions(currentConfig, true, false);
            std::cout << "Error is : " << error.x << ", " << -error.y << ", " << error.z << std::endl;
            return error;
        }
      }
    }
    ROS_ERROR_STREAM("Could not find the object we were checking for grasp error!");
    return error;
    
}

std::tuple<double, moveit_msgs::RobotTrajectory> kittingRobot::getCartesianPath(geometry_msgs::Pose initial_pose, geometry_msgs::Pose final_pose, bool collisionCheck){
    std::vector<geometry_msgs::Pose> waypoints = {initial_pose, final_pose};
    // move_group_interface->setMaxVelocityScalingFactor(0.05);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    std::cout << "Cartesian fraction = " << fraction << ", size of cartesian traj = " << trajectory.joint_trajectory.points.size() << std::endl;
    // move_group_interface->setMaxVelocityScalingFactor(1);
    return std::make_tuple(fraction, trajectory);
}

// for picking : Move towards object - (pan, y). pre_grasp_pose.
// Get base_link TF. Find if object is at front/back of robot.
// Move to home config front. 
bool kittingRobot::PickAndPlace(std::string ID, geometry_msgs::Pose initialPose, geometry_msgs::Pose finalPose, double offset){
    bool success;
    move_group_interface->setMaxVelocityScalingFactor(1);
    // std::cout << "Initial position of object is : " << initialPose.position.y << std::endl;
    double x_base = getTransform("base_link").transform.translation.x;
    ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
    feedback.feedback = feedback.MOVING_TO_OBJECT;
    pick_as->publishFeedback(feedback);
    // _______std::cout << "Moving towards object!" << std::endl;
    bool initialFront = (initialPose.position.x > x_base);
    success = moveTowardsObject(initialPose, initialFront, true, false);
    // pan tilt : 
    // if +ve and object at back, move to pi (back home) and to y simultaneously?
    // if -ve and object at back, move to -pi (back home) and to y simultaneously?
    // if +ve and object at front : if > pi then move to 2*pi. if < pi, move to 0. (front home both cases) Also move to y simultaneously
    // if -ve and object at front : if > -pi then move to 0. 
    // But moving toward joint limits is highly undesirable. Hence the above stuff is probs useless.
    // Just go to 0 if object at front, pi if object at back. Also go to y simultaneously.
    
    // if (!moveToHomeConfig(true, true)){
    //   ROS_ERROR_STREAM("Could not move to home config!");
    // }
    if (pick_as->isPreemptRequested() || !_nh->ok()){
      ROS_INFO("Static pick and place preempted");
      pick_as->setPreempted();
      return false;
    }

    // std::vector<double> cur_jvs = move_group_interface->getCurrentJointValues();
    // cur_jvs[0] = initialPose.position.y;
    // if(!publishToJointPositions(cur_jvs, 1)){
    //   ROS_ERROR_STREAM("Could not publish to y position!");
    // }
    // if (pick_as->isPreemptRequested() || !_nh->ok()){
    //   ROS_INFO("Static pick and place preempted");
    //   pick_as->setPreempted();
    //   return false;
    // }

    // if (!moveToHomeConfig(false, true)){
    //   ROS_ERROR_STREAM("Could not move to back home config!");
    // }
    // if (pick_as->isPreemptRequested() || !_nh->ok()){
    //   ROS_INFO("Static pick and place preempted");
    //   pick_as->setPreempted();
    //   return false;
    // }

    // std::vector<moveit_msgs::Grasp> grasps;
    // grasps.resize(1);
    // Setting grasp pose
    // grasps[0].grasp_pose.header.frame_id = move_group_interface->getPlanningFrame();
    // tf2::Quaternion grasp_pose_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).x + M_PI),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    // grasps[0].grasp_pose.pose.orientation = tf2::toMsg(grasp_pose_orientation);
    // grasps[0].grasp_pose.pose.position = initialPose.position;
    // grasps[0].grasp_pose.pose.position.z += offset;
    // // Setting pregrasp approach
    // grasps[0].pre_grasp_approach.direction.header.frame_id = "ee_link";
    // grasps[0].pre_grasp_approach.direction.vector.x = 1;
    // grasps[0].pre_grasp_approach.min_distance = offset - 0.125;
    // grasps[0].pre_grasp_approach.desired_distance = offset;
    // // Pose grasp retreat
    // grasps[0].post_grasp_retreat.direction.header.frame_id = "ee_link";
    // grasps[0].post_grasp_retreat.direction.vector.x = -1;
    // grasps[0].post_grasp_retreat.min_distance = offset;
    // grasps[0].post_grasp_retreat.desired_distance = offset + 0.125;
    // move_group_interface->setSupportSurfaceName(supporting_surface);
    // move_group_interface->pick(ID);
    // Move to y position first in home config
    // _________std::cout << "Attempting to move to pre_grasp_pose!" << std::endl;
    ros::Subscriber gripper_sub = _nh->subscribe("arm/gripper/state", 5, &kittingRobot::gripperStateCb, this);
    feedback.feedback = feedback.PREGRASP;
    pick_as->publishFeedback(feedback);
    tf2::Quaternion pre_grasp_orientation;
    if (initialFront)
      pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z),0,0)});
    else
      pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z + M_PI),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    geometry_msgs::Pose pre_grasp_pose, grasp_pose_estimate;
    pre_grasp_pose.orientation = tf2::toMsg(pre_grasp_orientation);
    pre_grasp_pose.position = initialPose.position;
    pre_grasp_pose.position.z += offset;
    success = moveToPoseWhileCM(pre_grasp_pose);
    if (pick_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_as->setPreempted();
        return false;
    }
    /*while ( (moveGroupFeedback.status != 3) && moveGroupFeedback.prev_state!="MONITOR"){
      if (pick_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_as->setPreempted();
        return false;
      }
    }*/
    std::cout << "moveToPose completed for pre_grasp" << std::endl;
    nist_gear::VacuumGripperControl gripper_srv;
    gripper_srv.request.enable = true;
    if (!gripperControl.call(gripper_srv)){
      ROS_ERROR_STREAM("Unable to turn on Gripper!");
      return false;
    }
    grasp_pose_estimate = pre_grasp_pose;
    grasp_pose_estimate.position.z -= (offset - 0.01);
    moveit_msgs::RobotTrajectory cartesian_traj_approach;
    double approach_fraction, placeOffsetZ;
    feedback.feedback = feedback.PROBE_AND_PICK;
    pick_as->publishFeedback(feedback);
    int count = 0;
    while (!objectAttached && count < 3){
      std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(pre_grasp_pose, grasp_pose_estimate);
      if ( approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0 ){
        std::cout << "Cartesian path found for approach!" << std::endl;
        probe(cartesian_traj_approach);
        std::cout << "Exited probe!" << std::endl;
        placeOffsetZ = (pre_grasp_pose.position.z - move_group_interface->getCurrentPose().pose.position.z) - 0.05;
        success = moveToPoseWhileCM(pre_grasp_pose);
      }
      else{
        ROS_ERROR_STREAM("Cartesian path fraction less than 1! Cannot pick!");

        return false;
      }
      if (objectAttached){
        std::vector<double> dimensions;
        geometry_msgs::Pose collision_pose;
        dimensions.resize(3);
        geometry_msgs::TransformStamped Transform = getTransform("gantry_arm_vacuum_gripper_link");
        collision_pose.position.x = Transform.transform.translation.x;
        collision_pose.position.y = Transform.transform.translation.y;
        collision_pose.position.z = Transform.transform.translation.z - 0.064;
        std::string id = "temp_collision";
        dimensions[0] = 0.128;
        dimensions[1] = 0.128;
        dimensions[2] = 0.128;
        collision_pose.orientation.w = 1;
        addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
        move_group_interface->attachObject("temp_collision");
      }
      count++;
    }
    // Start a callback to see if object remains attached to the gripper

    if (pick_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_as->setPreempted();
        return false;
    }

    feedback.feedback = feedback.MOVE_TO_GOAL;
    pick_as->publishFeedback(feedback);

    //  -------------- PLACE -------------------
    x_base = getTransform("base_link").transform.translation.x;
    bool finalFront = (finalPose.position.x > x_base);
    // moveTowardsObject(finalPose, finalFront, true);
    if (!moveTowardsObject(finalPose, finalFront, true, true) && !objectAttached){
      objectAttached = true;
      ROS_ERROR_STREAM("moveTowardsObject while GM failed! Object not attached!");
      gripper_sub.shutdown();
      return false;
    }
    feedback.feedback = feedback.PLACE;
    pick_as->publishFeedback(feedback);
    std::cout << "Attempting to move to pre_place_pose!" << std::endl;
    tf2::Quaternion pre_place_orientation;
    if (finalFront)
      pre_place_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(finalPose.orientation).z),0,0)});
    else
      pre_place_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(finalPose.orientation).z + M_PI),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    geometry_msgs::Pose pre_place_pose, place_pose;
    pre_place_pose.orientation = tf2::toMsg(pre_place_orientation);
    pre_place_pose.position = finalPose.position;
    // success = moveTowardsObject(finalPose, finalFront, true);
    pre_place_pose.position.z += (offset - placeOffsetZ + 0.01);

    if (!moveToPoseWhileCMandGM(pre_place_pose) && !objectAttached){
      objectAttached = true;
      ROS_ERROR_STREAM("moveToPose while GM failed! Object not attached!");
      gripper_sub.shutdown();
      return false;
    }

    planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"temp_collision"});
    
    gripper_srv.request.enable = false;
    if (!gripperControl.call(gripper_srv)){
      ROS_ERROR_STREAM("Unable to turn off Gripper!");
      return false;
    }

    success = moveToHomeConfig(finalFront, true, true);

    // if (pick_as->isPreemptRequested() || !_nh->ok()){
    //     ROS_INFO("Static pick and place preempted");
    //     pick_as->setPreempted();
    //     return false;
    // }

    /*std::cout << "moveToPose completed for pre_place" << std::endl;
    place_pose = pre_place_pose;
    place_pose.position.z -= placeOffsetZ;
    std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(pre_place_pose, place_pose);
    if ( approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0 ){
      std::cout << "Cartesian path found for approach!" << std::endl;
      move_group_interface->execute(cartesian_traj_approach);
      gripper_srv.request.enable = false;
      if (!gripperControl.call(gripper_srv)){
        ROS_ERROR_STREAM("Unable to turn off Gripper!");
      }
      // success = moveToPose(pre_place_pose, true);
      success = moveToHomeConfig(finalFront, true);
    }
    gripper_sub.shutdown();*/
    // geometry_msgs::PoseStamped actual_grasp_pose = move_group_interface->getCurrentPose();
    // std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(actual_grasp_pose.pose, pre_grasp_pose);
    // retract();

    return success;
}

bool kittingRobot::conveyorPickAndPlace(geometry_msgs::Pose initialPose, ros::Time lastPoseUpdateTime, geometry_msgs::Pose finalPose, double objectHeight, double offset){
    bool success;
    move_group_interface->setMaxVelocityScalingFactor(1);
    
    // std::cout << "Initial position of object is : " << initialPose.position.y << std::endl;
    double x_base = getTransform("base_link").transform.translation.x;
    ariac_2021_submission::kittingPickUpConveyorObjectFeedback feedback;
    feedback.feedback = feedback.MOVING_TO_OBJECT;
    pick_conveyor_as->publishFeedback(feedback);
    // _______std::cout << "Moving towards object!" << std::endl;
    // initialPose.position.y -= 0.2*(ros::Time::now() - lastPoseUpdateTime).toSec();
    // success = moveTowardsConveyorObject(initialPose, lastPoseUpdateTime, 1);

    if (pick_conveyor_as->isPreemptRequested() || !_nh->ok()){
      ROS_INFO("Conveyor pick and place preempted");
      pick_conveyor_as->setPreempted();
      return false;
    }
    
    ros::Subscriber gripper_sub = _nh->subscribe("arm/gripper/state", 5, &kittingRobot::gripperStateCb, this);
    
    tf2::Quaternion pre_grasp_orientation;
    // if (initialFront)
    pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z),0,0)});
    // else
    //   pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z + M_PI),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    geometry_msgs::Pose pre_grasp_pose, grasp_pose_estimate;
    pre_grasp_pose.orientation = tf2::toMsg(pre_grasp_orientation);
    pre_grasp_pose.position = initialPose.position;
    pre_grasp_pose.position.y = initialPose.position.y - 0.2*(ros::Time::now() - lastPoseUpdateTime).toSec() - 0.6;
    pre_grasp_pose.position.z += (double(objectHeight/2.0) + 0.05);
    if (pre_grasp_pose.position.y < -4){
      ROS_ERROR_STREAM("Object crossed conveyor! Cannot pick!");
      moveToHomeConfig(true, true, true);
      return false;
    }
    success = moveTowardsConveyorObject(initialPose, lastPoseUpdateTime, 1);
    // if ((initialPose.position.y - 0.2*(ros::Time::now() - lastPoseUpdateTime).toSec() - 0.6) < -4){
    //   ROS_ERROR_STREAM("Object crossed conveyor! Cannot pick!");
    //   moveToHomeConfig(true, true, true);
    //   return false;
    // }
    feedback.feedback = feedback.PREGRASP;
    pick_conveyor_as->publishFeedback(feedback);
    pre_grasp_pose.position.y = initialPose.position.y - 0.2*(ros::Time::now() - lastPoseUpdateTime).toSec() - 0.6;
    if (pre_grasp_pose.position.y < -4){
      ROS_ERROR_STREAM("Object crossed conveyor! Cannot pick!");
      moveToHomeConfig(true, true, true);
      return false;
    }
    success = moveToPoseWhileCM(pre_grasp_pose);
    if (pick_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_as->setPreempted();
        return false;
    }
    
    std::cout << "moveToPose completed for pre_grasp" << std::endl;
    
    nist_gear::VacuumGripperControl gripper_srv;
    gripper_srv.request.enable = true;
    if (!gripperControl.call(gripper_srv)){
      ROS_ERROR_STREAM("Unable to turn on Gripper!");
      return false;
    }
    grasp_pose_estimate = pre_grasp_pose;
    grasp_pose_estimate.position.z -= 0.051;
    moveit_msgs::RobotTrajectory cartesian_traj_approach;
    double approach_fraction, placeOffsetZ;
    // while ( grasp_pose_estimate.position.y - (initialPose.position.y - 0.2*(ros::Time::now()-lastPoseUpdateTime).toSec()) > 0.4){
    //   ros::spinOnce();
    // }
    std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(pre_grasp_pose, grasp_pose_estimate, false);
    int count = 0;
    while (!objectAttached && count < 3){
      if (count > 0){
        pre_grasp_pose.position.y = initialPose.position.y - 0.2*(ros::Time::now() - lastPoseUpdateTime).toSec() - 0.3;
        if (pre_grasp_pose.position.y < -4.3){
          ROS_ERROR_STREAM("Object crossed conveyor! Cannot pick!");
          moveToHomeConfig(true, true, true);
          return false;
        }
        success = moveToPoseWhileCM(pre_grasp_pose);
        grasp_pose_estimate = pre_grasp_pose;
        grasp_pose_estimate.position.z -= 0.051;
        std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(pre_grasp_pose, grasp_pose_estimate, false);
      }
      if ( approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0 ){
        std::cout << "Cartesian path found for approach!" << std::endl;
        feedback.feedback = feedback.PROBE_AND_PICK;
        pick_conveyor_as->publishFeedback(feedback);
        probeConveyor(cartesian_traj_approach, objectHeight, initialPose, lastPoseUpdateTime);
        std::cout << "Exited probe!" << std::endl;
        // placeOffsetZ = (pre_grasp_pose.position.z - move_group_interface->getCurrentPose().pose.position.z) - 0.05;
        
        // success = moveToPose(pre_grasp_pose, true);
        // success = moveToHomeConfig(true, true);
      }
      else{
        ROS_ERROR_STREAM("Cartesian path fraction less than 1! Cannot pick!");

        return false;
      }
      // ros::Duration(0.5).sleep();
      if (objectAttached){
        std::vector<double> dimensions;
        geometry_msgs::Pose collision_pose;
        dimensions.resize(3);
        geometry_msgs::TransformStamped Transform = getTransform("gantry_arm_vacuum_gripper_link");
        collision_pose.position.x = Transform.transform.translation.x;
        collision_pose.position.y = Transform.transform.translation.y;
        collision_pose.position.z = Transform.transform.translation.z - 0.064;
        std::string id = "temp_collision";
        dimensions[0] = 0.128;
        dimensions[1] = 0.128;
        dimensions[2] = 0.128;
        collision_pose.orientation.w = 1;
        addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
        move_group_interface->attachObject("temp_collision");
        ros::Duration(0.5).sleep();
      }
      count++;
    }
    success = moveToHomeConfig(true, true, true);
    
    if (count > 3 || !objectAttached){
      ROS_ERROR_STREAM("Either couldn't pick object or object de-attached");
      return false;
    }

    if (pick_conveyor_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_conveyor_as->setPreempted();
        return false;
    }

    feedback.feedback = feedback.MOVE_TO_GOAL;
    pick_conveyor_as->publishFeedback(feedback);

    //  -------------- PLACE -------------------
    x_base = getTransform("base_link").transform.translation.x;
    bool finalFront = (finalPose.position.x > x_base);
    // moveTowardsObject(finalPose, finalFront, true);
    if (!moveTowardsObject(finalPose, finalFront, true, true) && !objectAttached){
      objectAttached = true;
      gripper_sub.shutdown();
      return false;
    }
    feedback.feedback = feedback.PLACE;
    pick_conveyor_as->publishFeedback(feedback);
    std::cout << "Attempting to move to pre_place_pose!" << std::endl;
    ROS_ERROR_STREAM("Desired finalPose orientation is : " << rpyFromQuat(finalPose.orientation).z);
    tf2::Quaternion pre_place_orientation, cam_check_orientation;
    // cam_check_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(M_PI),0,0),RPY(0,-(M_PI),0)});
    if (finalFront)
      pre_place_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0), RPY(-rpyFromQuat(finalPose.orientation).z,0,0)});
    else
      pre_place_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY((M_PI-rpyFromQuat(finalPose.orientation).z),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    cam_check_orientation = pre_place_orientation;
    geometry_msgs::Pose pre_place_pose, cam_check_pose;
    pre_place_pose.orientation = tf2::toMsg(pre_place_orientation);
    pre_place_pose.position = finalPose.position;
    // success = moveToPose(finalPose, finalFront, true);
    // pre_place_pose.position.z += (double(objectHeight)/2 + 0.05);
    pre_place_pose.position.z += (double(objectHeight)/2 + 0.1);
    cam_check_pose.position = transformToPose(getCamTransform(finalPose)).position;
    cam_check_pose.position.z -= 0.55;
    cam_check_pose.orientation = tf2::toMsg(cam_check_orientation);
    success = moveToPoseWhileCMandGM(cam_check_pose);
    std::vector<double> currentConfig = move_group_interface->getCurrentJointValues();
    currentConfig[5] += M_PI;
    moveToJointPositionsWhileCMandGM(currentConfig);
    geometry_msgs::PoseStamped currentPose = move_group_interface->getCurrentPose();
    geometry_msgs::Vector3 rpy_expected/* = rpyFromQuat(currentPose.pose.orientation)*/;
    if (finalFront)
      cam_check_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(0,0,M_PI)/*,RPY(-rpyFromQuat(finalPose.orientation).z,0,0)*/});
    else
      cam_check_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY((M_PI),0,0),RPY(0,0,M_PI)/*,RPY(-rpyFromQuat(finalPose.orientation).z,0,0)*/});
    currentPose.pose.orientation = tf2::toMsg(cam_check_orientation);
    success = moveToPoseWhileCMandGM(currentPose.pose);
    // publishToJointPositions(currentConfig, 1);
    currentPose = move_group_interface->getCurrentPose();
    if (success)
      std::cout << "Success moving to desired pose! Rectifying final place pose using camera!" << std::endl;
    else
      std::cout << "Rectifying final place pose using camera!" << std::endl;
    geometry_msgs::Point error = getObjectGraspError(currentPose.pose);
    pre_place_pose.position.x += error.x;
    pre_place_pose.position.y -= error.y;
    rpy_expected = rpyFromQuat(pre_grasp_pose.orientation);
    pre_grasp_pose.orientation = quatFromRPY(rpy_expected.x, rpy_expected.y, rpy_expected.z - error.z);
    std::cout << "Rectified YAW is : " << rpyFromQuat(pre_grasp_pose.orientation).z << std::endl;
    success = moveToPoseWhileCMandGM(pre_place_pose);
    planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"temp_collision"});
    gripper_srv.request.enable = false;
    if (!gripperControl.call(gripper_srv)){
      ROS_ERROR_STREAM("Unable to turn off Gripper!");
      return false;
    }

    success = moveToHomeConfig(finalFront, true, true);
    // geometry_msgs::PoseStamped actual_grasp_pose = move_group_interface->getCurrentPose();
    // std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(actual_grasp_pose.pose, pre_grasp_pose);
    // retract();
    
    return success;
}

void kittingRobot::probe(moveit_msgs::RobotTrajectory trajectory, double vel_scale, double acc_scale){
    // finishedProbe = false;
    std::cout << "Probe called" << std::endl;
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", *_nh, ros::Duration(5));
    wrist_1_joint_index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), "wrist_1_joint"));
    robot_trajectory::RobotTrajectory rt(move_group_interface->getRobotModel(), joint_model_group);
    rt.setRobotTrajectoryMsg(*move_group_interface->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, vel_scale, acc_scale);
    // moveit_msgs::RobotTrajectory current_traj;
    // current_traj.joint_trajectory.header = trajectory.joint_trajectory.header;
    // current_traj.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    // wrist_1_joint_index = std::distance(trajectory.joint_trajectory.joint_names.begin(), std::find(trajectory.joint_trajectory.joint_names.begin(), trajectory.joint_trajectory.joint_names.end(), "wrist_1_joint"));
    // double desired_effort;
    finishedProbe = false;
    joint_states_sub = _nh->subscribe<sensor_msgs::JointState>("joint_states", 10, &kittingRobot::jointStatesCallback, this);
    control_msgs::FollowJointTrajectoryGoal goal;
    // goal.goal_time_tolerance = ros::Duration(10);
    moveit_msgs::RobotTrajectory modified_traj;
    rt.getRobotTrajectoryMsg(modified_traj);
    goal.trajectory = modified_traj.joint_trajectory;
    
    // for (auto i : goal.trajectory.points){
    //   i.time_from_start = ros::Duration(i.time_from_start.toSec()*25);
    // }
    // ros::Rate rate(100);
    follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
    
    // follow_jt_traj_client->waitForResult();
    ros::Time start_time = ros::Time::now();
    while (!finishedProbe){
      ros::spinOnce();
      if ((ros::Time::now() - start_time).toSec() > 15){
        ROS_ERROR_STREAM("Taking more than 15 secs!");
        break;
      }
    }
    std::cout << "FJT either cancelled or succeeded inside probe! LOL!" << std::endl;

    std::cout << "Sending empty traj to lift a bit!" << std::endl;
    joint_states_sub.shutdown();
    sendEmptyTrajectoryToFJTClient();
    // while (follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::PENDING ||
    // follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::ACTIVE ){
    //   ros::spinOnce();
    //   rate.sleep();
    // }

    std::cout << "End of probe!" << std::endl;

    /*current_traj.joint_trajectory.points.resize(2);
    current_traj.joint_trajectory.points[0] = trajectory.joint_trajectory.points[0];
    for (std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator i=trajectory.joint_trajectory.points.begin()+1;
      i!=trajectory.joint_trajectory.points.end(); i++){
        current_traj.joint_trajectory.points[1] = *i;
        // desired_effort = (*i).effort[wrist_1_joint_index];
        goal.trajectory = current_traj.joint_trajectory;
        // follow_jt_traj_client->sendGoalAndWait(goal);
        // std::cout << "Goal done!" << std::endl;
        follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
        follow_jt_traj_client->waitForResult();
        std::cout << "Before while" << std::endl;
        while (follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::PENDING ||
        follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::ACTIVE ){
          std::cout << "Actual state inside while is : " << follow_jt_traj_client->getState().toString().c_str() << std::endl;
          // ros::spinOnce();
        }
        std::cout << "Actual state After while is : " << follow_jt_traj_client->getState().toString().c_str() << std::endl;
        std::cout << "After while" << std::endl;
        // ros::spin();
      // call follow_jt_traj action 
    }*/
    // move_group_interface->execute(trajectory);
}


void kittingRobot::probeConveyor(moveit_msgs::RobotTrajectory trajectory, double objectHeight, geometry_msgs::Pose pose, ros::Time previousUpdateTime){
    std::cout << "Conveyor probe called" << std::endl;
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", *_nh, ros::Duration(5));
    wrist_1_joint_index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), "wrist_1_joint"));
    robot_trajectory::RobotTrajectory rt(move_group_interface->getRobotModel(), joint_model_group);
    rt.setRobotTrajectoryMsg(*move_group_interface->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    double current_Y = pose.position.y - 0.2*(ros::Time::now()-previousUpdateTime).toSec();
    double time_remaining = (current_Y - move_group_interface->getCurrentPose().pose.position.y)/0.2 + 0.2;
    double vel_scale = trajectory.joint_trajectory.points[trajectory.joint_trajectory.points.size()].time_from_start.toSec()/time_remaining;
    iptp.computeTimeStamps(rt, vel_scale);
    // moveit_msgs::RobotTrajectory current_traj;
    // current_traj.joint_trajectory.header = trajectory.joint_trajectory.header;
    // current_traj.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    // wrist_1_joint_index = std::distance(trajectory.joint_trajectory.joint_names.begin(), std::find(trajectory.joint_trajectory.joint_names.begin(), trajectory.joint_trajectory.joint_names.end(), "wrist_1_joint"));
    // double desired_effort;
    joint_states_sub = _nh->subscribe<sensor_msgs::JointState>("joint_states", 10, &kittingRobot::jointStatesConveyorCallback, this);
    control_msgs::FollowJointTrajectoryGoal goal;
    // goal.goal_time_tolerance = ros::Duration(10);
    moveit_msgs::RobotTrajectory modified_traj;
    rt.getRobotTrajectoryMsg(modified_traj);
    goal.trajectory = modified_traj.joint_trajectory;
    double traversalTime = modified_traj.joint_trajectory.points[modified_traj.joint_trajectory.points.size()-1].time_from_start.toSec();
    
    while ( ((pose.position.y - 0.2*(ros::Time::now()-previousUpdateTime).toSec()) - move_group_interface->getCurrentPose().pose.position.y )/0.2 > (traversalTime - 0.1)){
      // std::cout << "Waiting to probe down!" << std::endl;
      ros::spinOnce();
    }
    // for (auto i : goal.trajectory.points){
    //   i.time_from_start = ros::Duration(i.time_from_start.toSec()*25);
    // }
    // ros::Rate rate(100);
    finishedProbe = false;
    follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
    
    // follow_jt_traj_client->waitForResult();
    ros::Time start_time = ros::Time::now();
    while (!finishedProbe /*&& ((ros::Time::now() - start_time) > ros::Duration(6))*/){
      ros::spinOnce();
      if ((ros::Time::now() - start_time).toSec() > 6){
        ROS_ERROR_STREAM("Taking more than 6 secs!");
        break;
      }
    }
    std::cout << "FJT either cancelled or succeeded inside probe! LOL!" << std::endl;

    std::cout << "Sending empty traj to lift a bit!" << std::endl;
    joint_states_sub.shutdown();
    sendConveyorEmptyTrajectoryToFJTClient();
    std::vector<double> currentConfig = move_group_interface->getCurrentJointValues();
    currentConfig[2] -= 0.1;
    publishToJointPositions(currentConfig, 0.2);
    // while(!finishedProbe){
    //   ros::spinOnce();
    // }
    std::cout << "FJT kitting either cancelled or succeeded inside probe! LOL!" << std::endl;
    // while (follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::PENDING ||
    // follow_jt_traj_client->getState() == actionlib::SimpleClientGoalState::ACTIVE ){
    //   ros::spinOnce();
    //   rate.sleep();
    // }

    
    std::cout << "End of probe!" << std::endl;
}


void kittingRobot::sendEmptyTrajectoryToFJTClient(){
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = move_group_interface->getCurrentJointValues();
    goal.trajectory.points[0].positions[2] -= 0.001;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    goal.trajectory.points[0].positions[4] += 0.008;   // prev 0.03
    goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.01;
    goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.1);
    follow_jt_traj_client->sendGoal(goal);
}

void kittingRobot::sendStopTrajectoryToFJTClient(){
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = move_group_interface->getCurrentJointValues();
    goal.trajectory.points[0].time_from_start = ros::Duration(0.01);
    // goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    follow_jt_traj_client->sendGoal(goal);
}


void kittingRobot::sendConveyorEmptyTrajectoryToFJTClient(){
    // ros::Duration(0.1).sleep();
    /*control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = move_group_interface->getJointNames();
    goal.trajectory.points.resize(2);
    goal.trajectory.points[0].positions = move_group_interface->getCurrentJointValues();
    // goal.trajectory.points[0].positions[2] += 0.01;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    goal.trajectory.points[0].positions[4] += 0.012;   // prev 0.03
    // goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    // goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.001;
    // goal.trajectory.points[0].positions[5] -= 0.15;
    goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.25);

    goal.trajectory.points[1].positions = goal.trajectory.points[0].positions;
    goal.trajectory.points[1].positions[2] -= 0.05;
    goal.trajectory.points[1].positions[5] -= 0.02;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    // goal.trajectory.points[1].positions[4] += 0.008;   // prev 0.03
    goal.trajectory.points[1].time_from_start = ros::Duration(0.5);*/
    
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = move_group_interface->getCurrentJointValues();
    // goal.trajectory.points[0].positions[2] += 0.01;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    goal.trajectory.points[0].positions[4] += 0.015;   // prev 0.03
    // goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    // goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.001;
    // goal.trajectory.points[0].positions[5] -= 0.15;
    // goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.2);
    follow_jt_traj_client->sendGoal(goal);
    ros::Duration(0.2).sleep();
    // follow_jt_traj_client->waitForResult(ros::Duration(2));
}


void kittingRobot::fjtFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // std::cout << "Diff between desired and actual joint efforts = " << abs(feedback->error.effort[wrist_1_joint_index]) << std::endl;
    
}

void kittingRobot::fjtDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    std::cout << "FJT finished with state : " << state.toString().c_str() << std::endl;
    std::cout << "Result is : " << (result->error_code == result->SUCCESSFUL) << std::endl;

}

void kittingRobot::fjtActiveCb(){
    std::cout << "FJT Goal just went active!" << std::endl;
}

void kittingRobot::fjtMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // std::cout << "Diff between desired and actual joint efforts = " << abs(feedback->error.effort[wrist_1_joint_index]) << std::endl;
    if (objectAttached){
      std::cout << "FJT Monitor feedback : Attached!" << std::endl;
      // also track gantry TF
    }
    else{
      follow_jt_traj_client->cancelGoal();
      ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
      feedback.feedback = feedback.GRIPPER_FAULT;
      pick_as->publishFeedback(feedback);
      gripperMonitoredFJTFailed = true;
      follow_jt_traj_client->cancelGoal();
      sendEmptyTrajectoryToFJTClient();
    }
}

void kittingRobot::fjtMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    std::cout << "FJT Monitor finished with state : " << state.toString().c_str() << std::endl;
    std::cout << "Result is : " << (result->error_code == result->SUCCESSFUL) << std::endl;

}

void kittingRobot::fjtMonitorActiveCb(){
    std::cout << "FJT Monitor Goal just went active!" << std::endl;
}

void kittingRobot::gripperStateCb(const nist_gear::VacuumGripperStateConstPtr& state){
    objectAttached = state->attached;
}


void kittingRobot::staticPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpStaticObjectGoalConstPtr &goal){
    std::cout << "staticPickAndPlace callback called!" << std::endl;
    ariac_2021_submission::kittingPickUpStaticObjectResult result;
    result.success = PickAndPlace(goal->ID, goal->initial_pose, goal->final_pose);
    std::cout << "Setting spp action result!" << std::endl;
    // pick_as->setSucceeded(result);
    if (result.success)
      pick_as->setSucceeded(result);
    else
      pick_as->setAborted(result);
    // goal->
}

void kittingRobot::conveyorPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpConveyorObjectGoalConstPtr &goal){
    std::cout << "conveyorPickAndPlace callback called!" << std::endl;
    ariac_2021_submission::kittingPickUpConveyorObjectResult result;
    currentObjectType = goal->type;
    result.success = conveyorPickAndPlace(goal->initial_pose, goal->lastPoseUpdateTime, goal->final_pose, goal->objectHeight);
    std::cout << "Setting cpp action result!" << std::endl;
    if (result.success){
      pick_conveyor_as->setSucceeded(result);
      std::cout << "Yess! Setting as succeeded!" << std::endl;
    }
    else{
      pick_conveyor_as->setAborted(result);
      std::cout << "Setting as aborted" << std::endl;
    }
    currentObjectType = "";
}

bool kittingRobot::moveTowardsObject(geometry_msgs::Pose pose, bool front, bool blocking, bool gripperMonitoring){
    std::vector<double> joint_group_positions = move_group_interface->getCurrentJointValues();
    std::vector<double> config;
    config.push_back(pose.position.y);
    if (front)
      config.insert(config.end(), home_config.begin(), home_config.end());
    else {
      config.push_back(M_PI);
      config.insert(config.end(), home_config.begin()+1, home_config.end());
    }
    bool success;
    if (blocking){
      if (gripperMonitoring)
        success = moveToJointPositionsWhileCMandGM(config);
      else
        success = moveToJointPositionsWhileCM(config);
    }
    else
      success = moveToJointPositions(config, blocking);
    return success;
}

bool kittingRobot::moveTowardsConveyorObject(geometry_msgs::Pose pose, ros::Time lastPoseUpdateTime, double frontDistance){
    std::vector<double> joint_group_positions = move_group_interface->getCurrentJointValues();
    std::vector<double> config;
    double dest_Y = pose.position.y;
    moveToHomeConfig(true, true);
    dest_Y -= ( 0.2*(ros::Time::now()-lastPoseUpdateTime).toSec() + frontDistance);  // +0.75 previously
    config.push_back(dest_Y);
    config.insert(config.end(), home_config.begin(), home_config.end());
    // bool success = publishToJointPositions(config, 1);
    move_group_interface->setMaxAccelerationScalingFactor(1);
    move_group_interface->setMaxVelocityScalingFactor(1);
    bool success = moveToJointPositionsWhileCM(config);
    // if (front)
      // config.insert(config.end(), home_config.begin(), home_config.end());
    // else {
    //   config.push_back(M_PI);
    //   config.insert(config.end(), home_config.begin()+1, home_config.end());
    // }
    // bool success = moveToJointPositions(config, blocking);
    std::cout << "moveTowardsConvObject returning success = " << success << std::endl;
    return success;
}

bool kittingRobot::moveTowardsObjectWhileMonitoring(geometry_msgs::Pose pose, bool front){
    std::vector<double> joint_group_positions = move_group_interface->getCurrentJointValues();
    std::vector<double> config;
    config.push_back(pose.position.y);
    if (front)
      config.insert(config.end(), home_config.begin(), home_config.end());
    else {
      config.push_back(M_PI);
      config.insert(config.end(), home_config.begin()+1, home_config.end());
    }
    bool success = moveToJointPositionsWhileCMandGM(config);
    /*move_group_interface->setJointValueTarget(config);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface->plan(plan);
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = plan.trajectory_.joint_trajectory;
    follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtMonitorDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtMonitorActiveCb, this), boost::bind(&kittingRobot::fjtMonitorFeedbackCb, this, _1));
    bool success = follow_jt_traj_client->waitForResult();
    if (!gripperMonitoredFJTFailed){
      std::cout << "YEEEHAAWWWWW! Reached object while monitoring gripper!" << std::endl;
      success = true;
    }
    else{
      std::cout << "Nuuuuu. Apparently FJT failed." << std::endl;
      success = false;
      gripperMonitoredFJTFailed = false;
    }*/
    return success;
}

bool kittingRobot::moveToHomeConfig(bool front, bool blocking, bool collisionMonitoring) {
    current_state = move_group_interface->getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    std::vector<double> config;
    std::cout << "LOLZ : " << joint_group_positions[0] << std::endl;
    config.push_back(joint_group_positions[0]);
    if (front)
      config.insert(config.end(), home_config.begin(), home_config.end());
    else {
      config.push_back(M_PI);
      config.insert(config.end(), home_config.begin()+1, home_config.end());
    }
    std::cout << "Printing goal config values : " << std::endl;
    for (auto i : config)
      std::cout << i << " ";
    std::cout << std::endl;
    bool success = moveToJointPositions(config, blocking, collisionMonitoring);
    return success;
}


bool kittingRobot::moveToPose(geometry_msgs::Pose target_pose, bool blocking, bool gripperMonitoring){
    std::cout << "Movetopose called!" << std::endl;
    bool success;
    if (blocking){
      if (gripperMonitoring)
        success = moveToPoseWhileCM(target_pose);
      else
        success = moveToPoseWhileCMandGM(target_pose);
    }
    else
    {  
      move_group_interface->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      int c = 0;
      while (!success && c<5){
        success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // if (success and blocking)
        //   success = (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success and !blocking)
          success = (move_group_interface->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        c++;
      }
      if (c >= 5){
        std::cout << "Could not find plan for pose : " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << std::endl;
      }
    }
    return success;
}

bool kittingRobot::moveToPoseWhileCM(geometry_msgs::Pose target_pose){
    move_group_interface->setPoseTarget(target_pose);
    bool success, stopped = true;

    while (stopped){
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        // if (tempCollisionAdded)
        std::tie(success, traj)  = getShortestOfNPaths(move_group_interface, 30);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given kitting group!");
          return false;
        }
        stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!follow_jt_traj_client->getState().isDone()){
          if (!distanceViolated)
            std::cout << "Distance not violated!" << std::endl;
          else{
            follow_jt_traj_client->cancelGoal();
            sendStopTrajectoryToFJTClient();
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          ros::Duration(0.03).sleep();
        }
      }
    }
    // if (!blocking){
    //   success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   std::cout << "Asyncmove() called. Success = " << success << std::endl;
    // }
    // else
    //   success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

bool kittingRobot::moveToPoseWhileCMandGM(geometry_msgs::Pose target_pose){
    move_group_interface->setPoseTarget(target_pose);
    bool success, stopped = true;
    gripperMonitoredFJTStatus = errorCode::SUCCESS;

    while (stopped){
      if (!objectAttached){
        ROS_ERROR_STREAM("Object got detached!!!!");
        gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
        break;
      }
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        // if (tempCollisionAdded)
        std::tie(success, traj)  = getShortestOfNPaths(move_group_interface, 30);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given kitting group!");
          gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
          return false;
        }
        stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!follow_jt_traj_client->getState().isDone()){
          if (objectAttached && !distanceViolated)
            std::cout << "Object attached and distance not violated!" << std::endl;
          else{
            follow_jt_traj_client->cancelGoal();
            sendStopTrajectoryToFJTClient();
            if (!objectAttached){
              ROS_ERROR_STREAM("Kitting gripper failed while performing gripper monitored motion");
              gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
            }
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          ros::Duration(0.03).sleep();
        }
      }
    }
    // if (!blocking){
    //   success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   std::cout << "Asyncmove() called. Success = " << success << std::endl;
    // }
    // else
    //   success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

bool kittingRobot::moveToJointPositions(std::vector<double> j_values, bool blocking, bool collisionMonitoring){
    move_group_interface->setJointValueTarget(j_values);
    bool success;
    if (!blocking){
      success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Asyncmove() called. Success = " << success << std::endl;
    }
    else{
      if (collisionMonitoring)
        success = moveToJointPositionsWhileCM(j_values);
      else
        success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
      // success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

bool kittingRobot::moveToJointPositionsWhileCM(std::vector<double> j_values){
    move_group_interface->setJointValueTarget(j_values);
    bool success, stopped = true;

    while (stopped){
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        // if (tempCollisionAdded)
        std::tie(success, traj)  = getShortestOfNPaths(move_group_interface, 30);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          std::cout << "Sending FJT goal for kitting with CM" << std::endl; 
          follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given kitting group!");
          return false;
        }
        stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!follow_jt_traj_client->getState().isDone()){
          if (!distanceViolated)
            std::cout << "Kitting distance not violated!" << std::endl;
          else{
            follow_jt_traj_client->cancelGoal();
            sendStopTrajectoryToFJTClient();
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          ros::Duration(0.03).sleep();
        }
      }
    }
    // if (!blocking){
    //   success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   std::cout << "Asyncmove() called. Success = " << success << std::endl;
    // }
    // else
    //   success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}

bool kittingRobot::moveToJointPositionsWhileCMandGM(std::vector<double> j_values){
    move_group_interface->setJointValueTarget(j_values);
    bool success, stopped = true;
    gripperMonitoredFJTStatus = errorCode::SUCCESS;

    while (stopped){
      if (!objectAttached){
        gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
        break;
      }
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        // if (tempCollisionAdded)
        std::tie(success, traj)  = getShortestOfNPaths(move_group_interface, 30);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          follow_jt_traj_client->sendGoal(goal, boost::bind(&kittingRobot::fjtDoneCb, this, _1, _2), boost::bind(&kittingRobot::fjtActiveCb, this), boost::bind(&kittingRobot::fjtFeedbackCb, this, _1));
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given kitting group!");
          gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
          return false;
        }
        stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!follow_jt_traj_client->getState().isDone()){
          if (objectAttached && !distanceViolated)
            std::cout << "Object attached and distance not violated!" << std::endl;
          else{
            follow_jt_traj_client->cancelGoal();
            sendStopTrajectoryToFJTClient();
            if (!objectAttached){
              ROS_ERROR_STREAM("Gripper failed while performing gripper monitored motion");
              gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
            }
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          ros::Duration(0.03).sleep();
        }
      }
    }
    // if (!blocking){
    //   success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   std::cout << "Asyncmove() called. Success = " << success << std::endl;
    // }
    // else
    //   success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    return success;
}


std::tuple<bool, moveit_msgs::RobotTrajectory> kittingRobot::getShortestOfNPaths(moveit::planning_interface::MoveGroupInterfacePtr interface, int count){
    std::cout << "Searching for shortest of " << count << " paths!" << std::endl;
    bool curr_success, success = false;
    moveit::planning_interface::MoveGroupInterface::Plan curr_plan;
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    for (int i = 0; i < count; i++){
      success = success || (curr_success = (interface->plan(curr_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS));
      if (curr_success)
        plans.push_back(curr_plan);
    }
    moveit_msgs::RobotTrajectory traj = getShortestPath(plans);
    return std::make_tuple(success, traj);
}

static double getPathConfigDistance(trajectory_msgs::JointTrajectory traj){
    if (traj.points.size()>0)
    {
      std::vector<double> dist_factors{1,1,1,0.5,0.2,0.1};
      double dx = 0;
      for (std::size_t i = 0; i < traj.points.size()-1; i++){
        for (std::size_t j = 0; j < traj.points[0].positions.size(); j++){
          dx += abs(traj.points[i].positions[j] - traj.points[i+1].positions[j])*dist_factors[j];
        }
      }
      return dx;
    }
    else
      return 0;
}

moveit_msgs::RobotTrajectory kittingRobot::getShortestPath(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans){
    double min_dist = 99999999, cur_dist;
    moveit_msgs::RobotTrajectory trajectory;
    if (plans.size()>0){
      min_dist = getPathConfigDistance(plans[0].trajectory_.joint_trajectory);
      trajectory = plans[0].trajectory_;
    }
    for (std::vector<moveit::planning_interface::MoveGroupInterface::Plan>::iterator it = plans.begin(); 
      it != plans.end(); it++){
        cur_dist = getPathConfigDistance(it->trajectory_.joint_trajectory);
        if (min_dist > cur_dist && cur_dist > 0){
          min_dist = cur_dist;
          trajectory = it->trajectory_;
        }
      }
    std::cout << "Minimum dist is : " << min_dist << std::endl;
    return trajectory;
}

/*void kittingRobot::movingFeedbackCallback(const moveit_msgs::MoveGroupActionFeedbackConstPtr& msg){
    moveGroupFeedback.status = msg->status.status;
    moveGroupFeedback.prev_state = moveGroupFeedback.state;
    moveGroupFeedback.state = msg->feedback.state;
}*/

void kittingRobot::track_gantry_timer_callback(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped t = getTransform("base_link", "torso_base");
    if (abs(t.transform.translation.x) < 1 && abs(t.transform.translation.y) < 1 ){
      distanceViolated = true;
    }
    else
      distanceViolated = false;
    // std::cout << "Tracking gantry timer! DistanceViolated is : " << distanceViolated << std::endl;
}

void kittingRobot::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg){
    // wrist_1_joint_index
    double effort = joint_state_msg->effort[wrist_1_joint_index];
    std::cout << " wrist_1_joint effort : " << effort /*<< ", finishedProbe : " << finishedProbe */<< std::endl;
    if (abs(effort) > 2.2){
      follow_jt_traj_client->cancelGoal();
      std::cout << "Goal cancelled!" << std::endl;
      // sendEmptyTrajectoryToFJTClient();
      finishedProbe = true;
    }
}

void kittingRobot::gantryNextGoalCb(const geometry_msgs::PoseConstPtr& pose){
    gantry_next_goal = *pose;
}

void kittingRobot::jointStatesConveyorCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg){
    // wrist_1_joint_index
    double effort = joint_state_msg->effort[wrist_1_joint_index];
    std::cout << " wrist_1_joint effort : " << effort /*<< ", finishedProbe : " << finishedProbe */<< std::endl;
    if (effort > 2.2){
      follow_jt_traj_client->cancelGoal();
      std::cout << "Goal cancelled!" << std::endl;
      sendConveyorEmptyTrajectoryToFJTClient();
      finishedProbe = true;
    }
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
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }

    {
      dimensions[0] = 0.6;
      dimensions[1] = 0.6;
      dimensions[2] = 0.55;
      std::vector<double> dim{0.01, 0.58, 0.12};
      for (int i=1; i<=8; i++){
        id = "bin" + std::to_string(i) + "_center_collision";
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tfBuffer.lookupTransform("world", "bin"+std::to_string(i)+"_frame", 
                                  ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        /*collision_pose.orientation.w = 1;
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y;
        collision_pose.position.z = 0.45;
        // std::cout << collision_pose.position.x << " " << collision_pose.position.y << " " << collision_pose.position.z << std::endl;
        addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
        
        collision_pose.position.z += ((dimensions[2]/2) + 0.06);
        id = "bin" + std::to_string(i) + "front_edge";
        collision_pose.orientation.w = 1;
        collision_pose.position.x = transformStamped.transform.translation.x - 0.3;
        collision_pose.position.y = transformStamped.transform.translation.y;
        addPlanningSceneCollisionObject(id, dim, collision_pose, collisionAction::ADD);
        id = "bin" + std::to_string(i) + "back_edge";
        collision_pose.orientation.w = 1;
        collision_pose.position.x = transformStamped.transform.translation.x + 0.3;
        collision_pose.position.y = transformStamped.transform.translation.y;
        addPlanningSceneCollisionObject(id, dim, collision_pose, collisionAction::ADD);
        id = "bin" + std::to_string(i) + "left_edge";
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y - 0.3;
        addPlanningSceneCollisionObject(id, dim, collision_pose, collisionAction::ADD);
        id = "bin" + std::to_string(i) + "right_edge";
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y + 0.3;
        addPlanningSceneCollisionObject(id, dim, collision_pose, collisionAction::ADD);*/
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface->getPlanningFrame();
        collision_object.id = "bin" + std::to_string(i) + "_collision";;
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = dimensions;
        collision_object.primitives.push_back(primitive);
        collision_pose.orientation = quatFromRPY(0,0,0);
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y;
        collision_pose.position.z = 0.45;
        collision_object.primitive_poses.push_back(collision_pose);

        primitive.dimensions = dim;
        collision_object.primitives.push_back(primitive);
        collision_pose.position.z += ((dimensions[2]/2) + 0.062);
        collision_pose.orientation.w = 1;
        collision_pose.position.x = transformStamped.transform.translation.x - 0.3;
        collision_pose.position.y = transformStamped.transform.translation.y;
        collision_object.primitive_poses.push_back(collision_pose);

        collision_object.primitives.push_back(primitive);
        collision_pose.orientation.w = 1;
        collision_pose.position.x = transformStamped.transform.translation.x + 0.3;
        collision_pose.position.y = transformStamped.transform.translation.y;
        collision_object.primitive_poses.push_back(collision_pose);

        collision_object.primitives.push_back(primitive);
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y - 0.3;
        collision_object.primitive_poses.push_back(collision_pose);

        collision_object.primitives.push_back(primitive);
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = transformStamped.transform.translation.x;
        collision_pose.position.y = transformStamped.transform.translation.y + 0.3;
        collision_object.primitive_poses.push_back(collision_pose);

        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        collisionObjects.push_back(collision_object);
        std::cout << "Adding " << collision_object.id << " into the world" << std::endl;
        planning_scene_interface.applyCollisionObjects(collisionObjects);
        
      }
    }

    std::cout << "Finished adding the objects" << std::endl;

}

bool kittingRobot::addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action){
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
    else if (Action == collisionAction::REMOVE)
      collision_object.operation = collision_object.REMOVE;
    else if (Action == collisionAction::MOVE)
      collision_object.operation = collision_object.MOVE;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.push_back(collision_object);
    std::cout << "Adding " << Id << " into the world" << std::endl;
    return planning_scene_interface.applyCollisionObjects(collisionObjects);


}

bool kittingRobot::publishToJointPositions(std::vector<double> joint_positions, double time_from_start){
    // jointPositionPublisher.publish()
    trajectory_msgs::JointTrajectory msg;
    std::cout << "Publishing to joint positions!" << std::endl;
    /*std::cout << "Current joint names are : " << std::endl;
    std::vector<std::string> joint_names;
    joint_names = move_group_interface->getJointNames();
    std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(std::cout << ", "));*/
    msg.joint_names.clear();
    msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("shoulder_pan_joint");
    msg.joint_names.push_back("shoulder_lift_joint");
    msg.joint_names.push_back("elbow_joint");
    msg.joint_names.push_back("wrist_1_joint");
    msg.joint_names.push_back("wrist_2_joint");
    msg.joint_names.push_back("wrist_3_joint");
    msg.points.resize(1);
    msg.points[0].positions = joint_positions;
    msg.points[0].time_from_start = ros::Duration(time_from_start);
    // ros::Rate rate(10);
    bool too_much_time;
    ros::Time start_time = ros::Time::now();
    while (!compareCurrentToTargetJointPositions(joint_positions, 0.05) && (too_much_time = (ros::Duration(10) > (ros::Time::now() - start_time))) ){
      jointPositionPublisher.publish(msg);
      ros::spinOnce();
      // rate.sleep();
    }
    if(!too_much_time){
      std::cout << "PublishToJointPositions returning true!" << std::endl;
      return true;
    }
    else{
      return false;
      ROS_ERROR_STREAM("Took more than 10 secs to publish to joint positions");
    }
}

bool kittingRobot::compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance){
    std::vector<double> cur = move_group_interface->getCurrentJointValues();
    for (int i = 0; i < target.size(); i++){
      if ( abs(cur[i] - target[i]) < tolerance )
        continue;
      else
        return false;
    }
    return true;
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


// NOTE : In jointStatesCallback IF we check for both "objectAttached" (especially) as well as 
// joint force threshold. Might lead to severe problems in the case of a faulty gripper. 

// Note : Not sure why, but UR placed both blue battery AND the red part in the same place. Need 
// to fix that issue FIRST THING