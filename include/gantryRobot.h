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
#include <std_srvs/Trigger.h>
//
#include <ariac_2021_submission/kittingPickUpStaticObjectAction.h>
#include <ariac_2021_submission/kittingPickUpConveyorObjectAction.h>
#include <ariac_2021_submission/inspectionAssemblyAction.h>


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nist_gear/VacuumGripperControl.h>
#include <nist_gear/VacuumGripperState.h>



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

bool isPositionDiffLesser(geometry_msgs::Vector3 t1, geometry_msgs::Point t2, double x_offset, double y_offset, double diff=0.1){
    if( (abs(t1.x - x_offset - t2.x) < diff) && (abs(t1.y - y_offset - t2.y) < diff) ){
      return true;
    }
    // std::cout << abs(t1.x - t2.x) << ", " << abs(t1.y - t2.y) << std::endl;
    return false;
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

double max(std::vector<double> nos){
    if (nos.size() > 0){
      double max = nos[0];
      for (auto i : nos){
        if (max < i)
          max = i;
      }
      return max;
    }
    else
      return -99999;
}

double min(std::vector<double> nos){
    if (nos.size() > 0){
      double min = nos[0];
      for (auto i : nos){
        if (min > i)
          min = i;
      }
      return min;
    }
    else
      return 99999;
}

double avg(std::vector<double> nos){
    double sum = 0;
    for (auto i : nos)
      sum += i;
    return sum/nos.size();
}

// struct Sphere{
//     double r;
//     geometry_msgs::Point center;
// };

geometry_msgs::Pose vectorToPose(geometry_msgs::Vector3 p){
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = p.z;
    pose.orientation = quatFromRPY(0,0,0);
    return pose;
}

geometry_msgs::Pose pointToPose(geometry_msgs::Point p){
    geometry_msgs::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = p.z;
    pose.orientation = quatFromRPY(0,0,0);
    return pose;
}

tf2::Quaternion getNetRotation(std::vector<RPY> rpy_vector){
    tf2::Quaternion net_quat, quat;
    net_quat.setRPY(0,0,0);
    for (auto i : rpy_vector){
      quat.setRPY(i.R, i.P, i.Y);
      net_quat = net_quat * quat;
    }
    return net_quat;
}

enum collisionAction{
    ADD, REMOVE, MOVE
};

enum moveGroup{
    FULL, ARM, TORSO
};

enum errorCode{
    SUCCESS, GRIPPER_FAILED, NO_PLAN_FOUND
};


class gantryRobot{
  public:
    explicit gantryRobot(ros::NodeHandlePtr& nh) : _nh(nh) {
      tfListenerPtr = boost::make_shared<tf2_ros::TransformListener>(tfBuffer);
      jointPositionPublisher = _nh->advertise<trajectory_msgs::JointTrajectory>("gantry_arm_controller/command", 10);
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      arm_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(arm_planning_group_name);
      torso_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(torso_planning_group_name);
      joint_model_group = arm_move_group_interface->getCurrentState()->getJointModelGroup(arm_planning_group_name);
      next_goal_publisher = _nh->advertise<geometry_msgs::Pose>("next_goal", 3, true);
      track_kitting_timer = _nh->createTimer(ros::Duration(0.025), &gantryRobot::track_kitting_timer_callback, this);
      // advertisePickAndPlaceAction();
      /*std::cout << "End effector link is : " << move_group_interface->getEndEffectorLink() << std::endl;
      std::cout << "Current joint names in the torso move_group_interface are : ";
      std::vector<std::string> joint_names;
      joint_names = torso_move_group_interface->getJointNames();
      // std::cout << joint_names.size() << "sz" << std::endl;
      std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(std::cout << ", "));
      std::cout << std::endl << "Current joint names in the torso move_group_interface are : ";
      joint_names = arm_move_group_interface->getJointNames();
      std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(std::cout << ", "));*/
      // std::cout << "Initial joint tolerance = " << move_group_interface->getGoalJointTolerance() << std::endl;
      // std::cout << "Initial pose tolerance = " << move_group_interface->getGoalPositionTolerance() << ", " << move_group_interface->getGoalOrientationTolerance() << std::endl;
      move_group_interface->setGoalJointTolerance(0.0001);
      // move_group_interface->setMaxVelocityScalingFactor(0.2);
      // torso_move_group_interface->setMaxAccelerationScalingFactor(0.2);
      // arm_move_group_interface->setMaxVelocityScalingFactor(0.5);
      // torso_move_group_interface->setMaxVelocityScalingFactor(0.2);
      // torso_move_group_interface->setMaxVelocityScalingFactor(0.5);
      // torso_move_group_interface->setMaxAccelerationScalingFactor(0.2);
      // move_group_interface->setGoalPositionTolerance(0.001);
      // move_group_interface->setGoalOrientationTolerance(0.01);
      // std::cout << "Press something to move to manual pose" << std::endl;
      // std::cin.get();
      // std::cout << "Starting pick and place" << std::endl;
      
      /*k = moveArmToHomeConfig(false);
      std::cout << "Returned from moveArmToHomeConfig. SUCCESS = " << k << std::endl;
      k = (moveTorsoToHomeConfig(true));
      std::cout << "Returned from moveTorsoToHomeConfig. SUCCESS = " << k << std::endl;*/
      
      /*geometry_msgs::TransformStamped bin1_transform = getTransform("bin1_frame");
      geometry_msgs::TransformStamped objectTransform = getTransform("logical_camera_1_2_assembly_regulator_red_1_frame");
      geometry_msgs::Pose p, p1;
      p.orientation.w = 1;
      p.orientation = quatFromRPY(0, M_PI_2, 0);
      p.position.x = -2.000;
      p.position.y = 3.280;
      p.position.z = 0.756;
      std::cout << "Position of object is : { " << p.position.x << ", " << p.position.y << ", " << p.position.z << " }" << std::endl;
      // p.position.x += -1.8;
      // p.position.y += 2.5;
      // p.position.x = -3.566;
      // p.position.y = 0.659;
      // p.position.z = 1.117;
      // p.orientation = quatFromRPY(2.454, 1.531, 0.051);
      std::cout << "Moving torso to bin 1!" << std::endl;
      k = moveTorsoToPose(p, 0.1, -0.5, true);
      std::cout << "moveTorsoToPose returned SUCCESS = " << k << std::endl;
      p1 = p;
      p.position.z += 0.15;
      k = moveArmToPose(p, true);
      // wrist joint 2 (jvs[4]) -= pi/2 might be needed
      // std::vector<double> jvs = arm_move_group_interface->getCurrentJointValues();
      // jvs[3] 
      std::cout << "moveArmToPose returned SUCCESS = " << k << std::endl;
      moveit_msgs::RobotTrajectory cartesian_traj_approach;
      double approach_fraction;
      std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(p, p1);
      if ( approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0 ){
        std::cout << "Cartesian path found for approach!" << std::endl;
      }
      else
        std::cout << "Cartesian path not found! :(" << std::endl;*/
      std::cout << "Initializing gantry stuff!" << std::endl;
      addInitialCollisionObjects();
      std::cout << "Moving to home position!" << std::endl;
      bool k;
      k = moveFullToHomeConfig(true);
      std::cout << "Returned from moveFullToHomeConfig. SUCCESS = " << k << std::endl;
      advertiseActions();
      advertiseServices();
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
    const std::string planning_group_name = "gantry_full";
    const std::string arm_planning_group_name = "gantry_arm";
    const std::string torso_planning_group_name = "gantry_torso";
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
    moveit::planning_interface::MoveGroupInterfacePtr arm_move_group_interface;
    moveit::planning_interface::MoveGroupInterfacePtr torso_move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Timer track_kitting_timer;
    bool distanceViolated = false;
    // bool stopped = false;
    void track_kitting_timer_callback(const ros::TimerEvent& e);
    const robot_state::JointModelGroup* joint_model_group;
    moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
    moveit::core::RobotStatePtr current_state;
    tf2_ros::Buffer tfBuffer;
    boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    ros::ServiceServer addPlanningSceneCollisionService;
    ros::ServiceServer addGantrySphereService;
    ros::ServiceClient gripperControl;
    ros::Subscriber joint_states_sub;
    ros::Publisher next_goal_publisher;
    int wrist_1_joint_index;
    bool objectAttached;
    bool finishedProbe = false;
    errorCode gripperMonitoredFJTStatus = errorCode::SUCCESS;
    void jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_state);
    void jointStatesConveyorCallback(const sensor_msgs::JointStateConstPtr& joint_state);
    void movingFeedbackCallback(const moveit_msgs::MoveGroupActionFeedbackConstPtr& msg);
    boost::shared_ptr<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpStaticObjectAction>> pick_as;
    boost::shared_ptr<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpConveyorObjectAction>> pick_conveyor_as;
    boost::shared_ptr<actionlib::SimpleActionServer<ariac_2021_submission::inspectionAssemblyAction>> inspection_assembly_as;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_jt_traj_client;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> follow_jt_traj_torso_client;
    // ros::ServiceServer moveToJointPositionsService;
    // ros::ServiceServer moveToJointPositionsService;
    std::vector<double> home_config = {-2, 0, 0, -M_PI_2, -0.75, 2, -M_PI_2, M_PI_2, 0};   // linear,
    std::vector<double> arm_pick_config = {-0.65, -0.55, 1.2, -0.9, 1.44, 3.31};
    // gantry_controller : {1. small_long_joint (x), 3. torso_base_main_joint (yaw), 2. torso_rail_joint (-y)} {-1.5, 0, 0}
    // small_long_joint = desired_world_x + 2; 
    // gantry_arm_controller : -pi/2, -0.75, 2, -pi/2, pi/2, 0
    // std::vector<double> pick_config = { };
    // double actualEffort;
    void fjtFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void fjtDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void fjtActiveCb();
    void fjtMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void fjtMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void fjtMonitorActiveCb();
    void fjtTorsoMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback);
    void fjtTorsoMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result);
    void fjtTorsoMonitorActiveCb();
    void advertisePickAndPlaceAction();
    void advertiseServices();
    void advertiseActions();
    bool addCollisionServiceCallback(ariac_2021_submission::addPlanningSceneCollisionRequest& req, ariac_2021_submission::addPlanningSceneCollisionResponse &res);
    bool addSphereServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse &res);
    bool addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    bool addCollisionSpheres();
    bool addPlanningCollisionsForGroup(moveGroup group, std::vector<double> j_values, collisionAction action = collisionAction::ADD);
    std::vector<double> sphereRadii = {0.9, 0.42, 0.36, 0.14, 0.21};
    bool addInitialCollisionObjects();
    void sendEmptyTrajectoryToFJTClient();
    void sendStopTrajectoryToFJTClient();
    void sendEmptyTrajectoryToFJTTorsoClient();
    void sendConveyorEmptyTrajectoryToFJTClient();
    geometry_msgs::TransformStamped getTransform(std::string frame_name, std::string origin_frame = "world");
    std::tuple<double, moveit_msgs::RobotTrajectory> getCartesianPath(geometry_msgs::Pose initial_pose, geometry_msgs::Pose final_pose, bool collisionCheck=false);
    double getDifferenceBetweenConfigs(std::vector<double>, std::vector<double>);
    void addPartialCollisionObject(std::string ID, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action);
    ros::Subscriber movingFeedbackSub;
    ros::Publisher jointPositionPublisher;
    // bool publishToJointPositions(std::vector<double> joint_positions, double time_from_start = 2);  // This is a blocking function!
    std::tuple<bool, moveit_msgs::RobotTrajectory> getShortestOfNPaths(moveit::planning_interface::MoveGroupInterfacePtr interface, int count);
    moveit_msgs::RobotTrajectory getShortestPath(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans);
    bool moveFullToPose(geometry_msgs::Pose target_pose, bool blocking = false);
    bool moveArmToPose(geometry_msgs::Pose target_pose, bool blocking = false);
    bool moveArmToPoseWhileCM(geometry_msgs::Pose target_pose);
    bool moveArmToPoseWhileCMandGM(geometry_msgs::Pose target_pose);
    bool moveTorsoToPose(geometry_msgs::Pose target_pose, double x_offset, double y_offset, double yaw, bool blocking = false, bool gripperMonitoring = false);
    bool moveTorsoToPoseWhileCM(geometry_msgs::Pose target_pose, double x_offset, double y_offset, double yaw, bool blocking = false);
    bool moveToJointPositions(std::vector<double>, moveGroup group, bool blocking = false);
    bool moveToJointPositionsWhileCM(std::vector<double>, moveGroup group);
    bool moveToJointPositionsWhileCMandGM(std::vector<double>, moveGroup group);
    bool moveTorsoToHomeConfig(bool blocking = false);
    bool moveArmToHomeConfig(bool blocking = false, bool gripperMonitoring = false);
    bool moveFullToHomeConfig(bool blocking = false);
    bool moveTowardsObject(geometry_msgs::Pose pose, bool front, bool blocking = false);
    bool moveTowardsConveyorObject(geometry_msgs::Pose pose, ros::Time lastPoseUpdateTime, double frontDistance = 1);
    errorCode moveTowardsObjectWhileGripperMonitoring(geometry_msgs::Pose pose);
    // void probe(geometry_msgs::Vector3 direction);
    bool publishToJointPositions(std::vector<double> joint_positions, double time_from_start = 2);  // This is a blocking function!
    bool PickAndPlace(std::string ID, geometry_msgs::Pose initialPose, geometry_msgs::Pose finalPose, double offset = 0.15);
    bool conveyorPickAndPlace(geometry_msgs::Pose initialPose, ros::Time lastPoseUpdateTime, geometry_msgs::Pose finalPose, double objectHeight, double offset = 0.15);
    void probe(moveit_msgs::RobotTrajectory trajectory, double vel_scale = 0.1, double acc_scale = 0.1);
    void probeConveyor(moveit_msgs::RobotTrajectory trajectory, geometry_msgs::Pose pose, ros::Time previousUpdateTime);
    bool compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance = 0.1);
    void staticPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpStaticObjectGoalConstPtr &goal);
    void conveyorPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpConveyorObjectGoalConstPtr &goal);
    void inspectionAssemblyCallback(const ariac_2021_submission::inspectionAssemblyGoalConstPtr &goal);
    void gripperStateCb(const nist_gear::VacuumGripperStateConstPtr& state);
    
    
};


void gantryRobot::advertiseActions(){
    pick_as = boost::make_shared<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpStaticObjectAction>>(*_nh, "gantry_pick_and_place_static_action", boost::bind(&gantryRobot::staticPickAndPlaceCallback, this, _1), false);
    // pick_conveyor_as = boost::make_shared<actionlib::SimpleActionServer<ariac_2021_submission::kittingPickUpConveyorObjectAction>>(*_nh, "gantry_pick_and_place_conveyor_action", boost::bind(&gantryRobot::conveyorPickAndPlaceCallback, this, _1), false);
    pick_as->start();
    inspection_assembly_as = boost::make_shared<actionlib::SimpleActionServer<ariac_2021_submission::inspectionAssemblyAction>>(*_nh, "inspection_assembly_action", boost::bind(&gantryRobot::inspectionAssemblyCallback, this, _1), false);
    inspection_assembly_as->start();
    // pick_conveyor_as->start();
    follow_jt_traj_client = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("gantry_arm_controller/follow_joint_trajectory");
    follow_jt_traj_torso_client = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("gantry_controller/follow_joint_trajectory");
}

void gantryRobot::advertiseServices(){
    addPlanningSceneCollisionService = _nh->advertiseService("add_gantry_planning_scene_collision", &gantryRobot::addCollisionServiceCallback, this);
    addGantrySphereService = _nh->advertiseService("add_gantry_sphere", &gantryRobot::addSphereServiceCallback, this);
    gripperControl = _nh->serviceClient<nist_gear::VacuumGripperControl>("arm/gripper/control");
    gripperControl.waitForExistence(ros::Duration(10));
}

bool gantryRobot::addCollisionServiceCallback(ariac_2021_submission::addPlanningSceneCollisionRequest& req, 
  ariac_2021_submission::addPlanningSceneCollisionResponse &res){
    res.success = addPlanningSceneCollisionObject(req.ID, req.Dimensions, req.CollisionPose, collisionAction::ADD);
    return res.success;
}

bool gantryRobot::addSphereServiceCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse &res){
    std::cout << "Add sphere service called!" << std::endl;
    if (addCollisionSpheres()){
      res.success = true;
      std::cout << "Yes! Successfully added spheres!" << std::endl;
    }
    else{
      res.success = false;
      std::cout << "Nope! Couldn't add the spheres!" << std::endl;
    }
    return res.success;
}

geometry_msgs::TransformStamped gantryRobot::getTransform(std::string frame_name, std::string origin_frame){
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


bool gantryRobot::addPlanningSceneCollisionObject(std::string Id, std::vector<double> Dimensions, geometry_msgs::Pose CollisionPose, collisionAction Action){
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


bool gantryRobot::moveTorsoToHomeConfig(bool blocking) {
    std::vector<double> config;
    config.insert(config.end(), home_config.begin(), home_config.begin()+3);
    bool success = moveToJointPositions(config, moveGroup::TORSO, blocking);
    return success;
}

bool gantryRobot::moveArmToHomeConfig(bool blocking, bool gripperMonitoring) {
    std::vector<double> config;
    config.insert(config.end(), home_config.begin()+3, home_config.end());
    bool success;
    if (getDifferenceBetweenConfigs(config, arm_move_group_interface->getCurrentJointValues()) < 0.1){
      std::cout << "Arm is already very near to home config! Returning true!";
      return true;
    }
    if (!blocking)
      success = moveToJointPositions(config, moveGroup::ARM, blocking);
    else if (gripperMonitoring)
      success = moveToJointPositionsWhileCMandGM(config, moveGroup::ARM);
    else
      success = moveToJointPositionsWhileCM(config, moveGroup::ARM);
    return success;
}

bool gantryRobot::moveFullToHomeConfig(bool blocking) {
    bool success = moveToJointPositions(home_config, moveGroup::FULL, blocking);
    return success;
}

bool gantryRobot::moveToJointPositions(std::vector<double> j_values, moveGroup group, bool blocking /*= false*/){
    moveit::planning_interface::MoveGroupInterfacePtr interface;
    bool tempCollisionAdded = false, shortestPath = false;
    std::vector<std::string> ids;
    ids.resize(4);
    std::vector<double> dimensions;
    geometry_msgs::Pose collision_pose;
    dimensions.resize(3);
    switch (group){
      case moveGroup::FULL :{
        interface = move_group_interface;
        break;
      }

      case moveGroup::ARM :{
        /*interface = move_group_interface;
        std::vector<double> jvs = torso_move_group_interface->getCurrentJointValues();
        j_values.insert(j_values.begin(), jvs.begin(), jvs.end());
        // std::cout << "Arm interface was passed" << std::endl;
        // for (auto i : j_values)
        //   std::cout << i << " ";
        // std::cout << std::endl;
        tempCollisionAdded = true;
        // std::vector<double> jvs = arm_move_group_interface->getCurrentJointValues();
        // j_values.insert(j_values.end(), jvs.begin(), jvs.end());
        std::vector<double> torso_jvs = move_group_interface->getCurrentJointValues();
        // Adding conveyor collisions
        ids[0] = "arm_planning_collision0";
        dimensions[0] = 0.01;
        dimensions[1] = 20;
        dimensions[2] = 10;
        collision_pose.orientation = quatFromRPY(0,0,0);
        collision_pose.position.x = j_values[0] - 2 - 1.5;
        collision_pose.position.y = 0;
        collision_pose.position.z = 5;
        addPlanningSceneCollisionObject(ids[0], dimensions, collision_pose, collisionAction::ADD);
        collision_pose.position.x = j_values[0] - 2 + 1.5;
        ids[1] = "arm_planning_collision1";
        addPlanningSceneCollisionObject(ids[1], dimensions, collision_pose, collisionAction::ADD);
        ids[2] = "arm_planning_collision2";
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = 0;
        collision_pose.position.y = -j_values[1] - 1.5;
        addPlanningSceneCollisionObject(ids[2], dimensions, collision_pose, collisionAction::ADD);
        ids[3] = "arm_planning_collision3";
        collision_pose.position.x = 0;
        collision_pose.position.y = -j_values[1] + 1.5;
        addPlanningSceneCollisionObject(ids[3], dimensions, collision_pose, collisionAction::ADD);*/
        tempCollisionAdded = true;
        interface = arm_move_group_interface;
           // Need to add these collisions only in the case of moveArmToPose momentarily planning
        ids.resize(2);
        ids[0] = "torso_tray_collision";
        dimensions[0] = 0.88;
        dimensions[1] = 0.77;
        dimensions[2] = 0.0192*2;
        collision_pose.orientation.w = 1;
        geometry_msgs::TransformStamped t = getTransform("torso_tray");
        collision_pose.position.x = t.transform.translation.x;
        collision_pose.position.y = t.transform.translation.y;
        collision_pose.position.z = t.transform.translation.z - 0.0192;
        addPlanningSceneCollisionObject(ids[0], dimensions, collision_pose, collisionAction::ADD);
      

      
        ids[1] = "torso_main_collision";  // 0.921, 0.698, 0.921. Transform of {0,0,0.774}
        dimensions[0] = 0.921;
        dimensions[1] = 0.698;
        dimensions[2] = 0.921;
        collision_pose.orientation.w = 1;
        t = getTransform("torso_main");
        collision_pose.position.x = t.transform.translation.x;
        collision_pose.position.y = t.transform.translation.y;
        collision_pose.position.z = t.transform.translation.z + 0.774*2;
        addPlanningSceneCollisionObject(ids[1], dimensions, collision_pose, collisionAction::ADD);
    
        break;
      }

      case moveGroup::TORSO :{
        // std::cout << "Torso interface was passed" << std::endl;
        // std::cout << "Torso interface was passed" << std::endl;
        // for (auto i : j_values)
        //   std::cout << i << " ";
        // std::cout << std::endl;
        interface = torso_move_group_interface;
        tempCollisionAdded = shortestPath = true;
        // std::vector<double> jvs = arm_move_group_interface->getCurrentJointValues();
        // j_values.insert(j_values.end(), jvs.begin(), jvs.end());
        std::vector<double> torso_jvs = move_group_interface->getCurrentJointValues();
        // Adding conveyor collisions
        ids[0] = "torso_planning_collision0";
        dimensions[0] = 0.01;
        dimensions[1] = 20;
        dimensions[2] = 2.8;
        collision_pose.orientation = quatFromRPY(0,0,0);
        collision_pose.position.x = min({j_values[0]-2, torso_jvs[0]-2}) - 2;
        collision_pose.position.y = 0;
        collision_pose.position.z = dimensions[2]/2;
        addPlanningSceneCollisionObject(ids[0], dimensions, collision_pose, collisionAction::ADD);
        collision_pose.position.x = max({j_values[0]-2, torso_jvs[0]-2}) + 2;
        ids[1] = "torso_planning_collision1";
        addPlanningSceneCollisionObject(ids[1], dimensions, collision_pose, collisionAction::ADD);
        ids[2] = "torso_planning_collision2";
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = 0;
        collision_pose.position.y = min({-j_values[1], -torso_jvs[1]}) - 2;
        addPlanningSceneCollisionObject(ids[2], dimensions, collision_pose, collisionAction::ADD);
        ids[3] = "torso_planning_collision3";
        collision_pose.position.x = 0;
        collision_pose.position.y = max({-j_values[1], -torso_jvs[1]}) + 2;
        addPlanningSceneCollisionObject(ids[3], dimensions, collision_pose, collisionAction::ADD);
        break;
      }
    }
    interface->setJointValueTarget(j_values);
    std::cout << "No of joints : " << interface->getJointNames().size() << std::endl;
    bool success;
    if (!blocking){
      if (!shortestPath){
        success = (interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (tempCollisionAdded){
          std::cout << "Removing torso planning collisions" << std::endl;
          // while (true){
          //   continue;
          // }
          for (auto i : ids){
            std::cout << "Beep : " << addPlanningSceneCollisionObject(i, dimensions, geometry_msgs::Pose(), collisionAction::REMOVE) << std::endl;
          }
          if (!success)
            success = (interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
        std::cout << "Asyncmove() called. Success = " << success << std::endl;
      }
      else{
        moveit_msgs::RobotTrajectory traj;
        std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        if (success)
          success = (interface->asyncExecute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (tempCollisionAdded){
          std::cout << "Removing torso planning collisions" << std::endl;
          for (auto i : ids){
            std::cout << "Beep : " << addPlanningSceneCollisionObject(i, dimensions, geometry_msgs::Pose(), collisionAction::REMOVE) << std::endl;
          }
          if (!success){
            std::tie(success, traj)  = getShortestOfNPaths(interface, 10);
            success = (interface->asyncExecute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);            
          }
        }
      }
    }
    else{
      if (!shortestPath){
        success = (interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (tempCollisionAdded){
          std::cout << "Removing torso planning collisions" << std::endl;
          for (auto i : ids){
            std::cout << "Beep : " << addPlanningSceneCollisionObject(i, dimensions, geometry_msgs::Pose(), collisionAction::REMOVE) << std::endl;
          }
          if (!success)
            success = (interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
      }
      else{
        moveit_msgs::RobotTrajectory traj;
        std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        if (success)
          success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (tempCollisionAdded){
          std::cout << "Removing torso planning collisions" << std::endl;
          for (auto i : ids){
            std::cout << "Beep : " << addPlanningSceneCollisionObject(i, dimensions, geometry_msgs::Pose(), collisionAction::REMOVE) << std::endl;
          }
          if (!success){
            std::tie(success, traj)  = getShortestOfNPaths(interface, 10);
            success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);            
          }
        }
      }
    }
    // while (true){
    //   continue;
    // }
    return success;
}


bool gantryRobot::moveToJointPositionsWhileCM(std::vector<double> j_values, moveGroup group){
    std::cout << "moveToJointPositionsWhileCM called!" << std::endl;
    moveit::planning_interface::MoveGroupInterfacePtr interface;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_client;
    // follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
    // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
    bool tempCollisionAdded = false;
    switch (group){
      case moveGroup::FULL :{
        ROS_ERROR_STREAM("moveToJointPositionsWhileCM does not work for full config!");
        return false;
      }

      case moveGroup::ARM :{
        tempCollisionAdded = true;
        interface = arm_move_group_interface;
        fjt_client = follow_jt_traj_client;
           // Need to add these collisions only in the case of moveArmToPose momentarily planning
        addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::ADD);
        break;
      }

      case moveGroup::TORSO :{
        // std::cout << "Torso interface was passed" << std::endl;
        // std::cout << "Torso interface was passed" << std::endl;
        // for (auto i : j_values)
        //   std::cout << i << " ";
        // std::cout << std::endl;
        interface = torso_move_group_interface;
        tempCollisionAdded = true;
        fjt_client = follow_jt_traj_torso_client;
        // std::vector<double> jvs = arm_move_group_interface->getCurrentJointValues();
        // j_values.insert(j_values.end(), jvs.begin(), jvs.end());
        addPlanningCollisionsForGroup(group, j_values, collisionAction::ADD);
        break;
      }
    }
    interface->setJointValueTarget(j_values);
    std::cout << "No of joints : " << interface->getJointNames().size() << std::endl;
    bool success, stopped = true;
    
    
    
    while (stopped){
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        if (tempCollisionAdded)
          std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        else
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          if (group == moveGroup::ARM){
            follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
            std::cout << "sent goal for arm!" << std::endl;
          }
          if (group == moveGroup::TORSO){
            follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
            std::cout << "sent goal for torso!" << std::endl;
          }
        }
          // success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        else if (tempCollisionAdded){
          std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
          tempCollisionAdded = false;
          addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
          if (success){
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = traj.joint_trajectory;
            if (group == moveGroup::ARM){
              fjt_client = follow_jt_traj_client;            
              follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
            }
            if (group == moveGroup::TORSO){
              fjt_client = follow_jt_traj_torso_client;
              follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
            }
          }
          else{
            ROS_ERROR_STREAM("Could not find plan for the given group even after removing group collisions!");
            return false;
          }
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given group even though group collisions already removed!");
          return false;
        }
        stopped = false;
        // std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
        // goal.trajectory = traj.joint_trajectory;
        // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
        // stopped = false;
      }
      // std::cout << "Waiting for just a bit for goal to start executing " << std::endl;
      // ros::Duration(0.1).sleep();
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!fjt_client->getState().isDone()){
          if (!distanceViolated)
            ros::Duration(0.01).sleep();
            // std::cout << "Distance not violated!" << std::endl;
          else{
            fjt_client->cancelGoal();
            if (group == moveGroup::TORSO)
              sendEmptyTrajectoryToFJTTorsoClient();
            else if (group == moveGroup::ARM)
              sendStopTrajectoryToFJTClient();
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          // ros::Duration(0.01).sleep();
        }
      }
    }

    if (tempCollisionAdded){
      // std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
      // tempCollisionAdded = false;
      addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
    }
    // while (true){
    //   continue;
    // }
    return success;
}


bool gantryRobot::moveToJointPositionsWhileCMandGM(std::vector<double> j_values, moveGroup group){
    std::cout << "moveToJointPositionsWhileCM called!" << std::endl;
    gripperMonitoredFJTStatus = errorCode::SUCCESS;
    moveit::planning_interface::MoveGroupInterfacePtr interface;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_client;
    // follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
    // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
    bool tempCollisionAdded = false;
    switch (group){
      case moveGroup::FULL :{
        ROS_ERROR_STREAM("moveToJointPositionsWhileCM does not work for full config!");
        return false;
      }

      case moveGroup::ARM :{
        tempCollisionAdded = true;
        interface = arm_move_group_interface;
        fjt_client = follow_jt_traj_client;
           // Need to add these collisions only in the case of moveArmToPose momentarily planning
        addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::ADD);
        break;
      }

      case moveGroup::TORSO :{
        // std::cout << "Torso interface was passed" << std::endl;
        // std::cout << "Torso interface was passed" << std::endl;
        // for (auto i : j_values)
        //   std::cout << i << " ";
        // std::cout << std::endl;
        interface = torso_move_group_interface;
        tempCollisionAdded = true;
        fjt_client = follow_jt_traj_torso_client;
        // std::vector<double> jvs = arm_move_group_interface->getCurrentJointValues();
        // j_values.insert(j_values.end(), jvs.begin(), jvs.end());
        addPlanningCollisionsForGroup(group, j_values, collisionAction::ADD);
        break;
      }
    }
    interface->setJointValueTarget(j_values);
    std::cout << "No of joints : " << interface->getJointNames().size() << std::endl;
    bool success, stopped = true;

    
    while (stopped){
      if (!objectAttached){
        gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
        break;
      }
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        if (tempCollisionAdded)
          std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        else
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          if (group == moveGroup::ARM){
            follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
          }
          if (group == moveGroup::TORSO){
            follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
          }
        }
          // success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        else if (tempCollisionAdded){
          std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
          tempCollisionAdded = false;
          addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
          if (success){
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = traj.joint_trajectory;
            if (group == moveGroup::ARM){
              fjt_client = follow_jt_traj_client;            
              follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
            }
            if (group == moveGroup::TORSO){
              fjt_client = follow_jt_traj_torso_client;
              follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
            }
          }
          else{
            ROS_ERROR_STREAM("Could not find plan for the given group even after removing group collisions!");
            gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
            return false;
          }
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the given group even though group collisions already removed!");
          gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
          return false;
        }
        stopped = false;
        // std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
        // goal.trajectory = traj.joint_trajectory;
        // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
        // stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!fjt_client->getState().isDone()){
          if (objectAttached && !distanceViolated)
            ros::Duration(0.01).sleep();
            // std::cout << "Object attached and distance not violated!" << std::endl;
          else{
            fjt_client->cancelGoal();
            if (group == moveGroup::TORSO)
              sendEmptyTrajectoryToFJTTorsoClient();
            else if (group == moveGroup::ARM)
              sendStopTrajectoryToFJTClient();
            if (!objectAttached){
              ROS_ERROR_STREAM("Gripper failed while performing gripper monitored motion");
              // ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
              // feedback.feedback = feedback.GRIPPER_FAULT;
              // pick_as->publishFeedback(feedback);
              gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
              // follow_jt_traj_torso_client->cancelGoal();
            }
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          // ros::Duration(0.01).sleep();
        }
      }
    }

    if (tempCollisionAdded){
      // std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
      // tempCollisionAdded = false;
      addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
    }
    // while (true){
    //   continue;
    // }
    return success;
}



bool gantryRobot::moveFullToPose(geometry_msgs::Pose target_pose, bool blocking){
    // std::cout << "Movetopose called!" << std::endl;
    move_group_interface->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int c = 0;
    while (!success && c<3){
      success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success and blocking)
        success = (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      else if (success and !blocking)
        success = (move_group_interface->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      c++;
    }
    if (c >= 3){
      std::cout << "Could not find plan for pose : " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << std::endl;
    }
    return success;
}

bool gantryRobot::moveTorsoToPose(geometry_msgs::Pose target_pose, double x_offset, double y_offset, double yaw, bool blocking, bool gripperMonitoring){
    // std::cout << "Movetopose called!" << std::endl;
    target_pose.position.x += (x_offset + 2);
    target_pose.position.y *= -1;
    target_pose.position.y -= y_offset;
    bool success;
    if (!blocking)
      success = moveToJointPositions(std::vector<double>{target_pose.position.x, target_pose.position.y, yaw /*rpyFromQuat(target_pose.orientation).z*/}, moveGroup::TORSO, blocking);
    else
      success = moveToJointPositionsWhileCM(std::vector<double>{target_pose.position.x, target_pose.position.y, yaw /*rpyFromQuat(target_pose.orientation).z*/}, moveGroup::TORSO);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // bool success = false;
    // int c = 0;
    // while (!success && c<5){
    //   success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   if (success and blocking)
    //     success = (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   else if (success and !blocking)
    //     success = (move_group_interface->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   c++;
    // }
    // if (c >= 5){
    //   std::cout << "Could not find plan for pose : " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << std::endl;
    // }
    return success;
}

bool gantryRobot::moveTorsoToPoseWhileCM(geometry_msgs::Pose target_pose, double x_offset, double y_offset, double yaw, bool blocking){
    // std::cout << "Movetopose called!" << std::endl;
    target_pose.position.x += (x_offset + 2);
    target_pose.position.y *= -1;
    target_pose.position.y -= y_offset;
    torso_move_group_interface->setJointValueTarget(std::vector<double>{target_pose.position.x, target_pose.position.y, yaw});
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (torso_move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success){
      // Create a subscriber to track kitting robot
      while (true){   // Follow the trajectory pt by pt
        
      }
    }
    success = moveToJointPositions(std::vector<double>{target_pose.position.x, target_pose.position.y, yaw /*rpyFromQuat(target_pose.orientation).z*/}, moveGroup::TORSO, blocking);
    return success;
}

bool gantryRobot::moveArmToPose(geometry_msgs::Pose target_pose, bool blocking){
    // std::cout << "Movetopose called!" << std::endl;
    // target_pose.position.x += x_offset;
    // target_pose.position.y += y_offset;
    // bool success = moveToJointPositions(std::vector<double>{target_pose.position.x, target_pose.position.y, rpyFromQuat(target_pose.orientation).z}, moveGroup::TORSO, true);
    // torso_tray is a cube with 0.77, 0.88, 0.0192
    // gantry_body is a cube with (torso_main) dimensions : 0.921, 0.698, 0.921. Transform of {0,0,0.774}
    std::cout << "Move arm to pose function called " << std::endl;
    std::string id1 = "torso_tray_collision";
    std::vector<double> dimensions1, dimensions2;
    dimensions1.resize(3);
    geometry_msgs::Pose collision_pose1, collision_pose2;
    dimensions1[0] = 0.88;
    dimensions1[1] = 0.77;
    dimensions1[2] = 0.0192*2;
    collision_pose1.orientation.w = 1;
    geometry_msgs::TransformStamped t = getTransform("torso_tray");
    collision_pose1.position.x = t.transform.translation.x;
    collision_pose1.position.y = t.transform.translation.y;
    collision_pose1.position.z = t.transform.translation.z - 0.0192;
    addPlanningSceneCollisionObject(id1, dimensions1, collision_pose1, collisionAction::ADD);

    dimensions2.resize(3);
    std::string id2 = "torso_main_collision";  
    dimensions2[0] = 0.921;
    dimensions2[1] = 0.698;
    dimensions2[2] = 0.921;
    collision_pose2.orientation.w = 1;
    t = getTransform("torso_main");
    collision_pose2.position.x = t.transform.translation.x;
    collision_pose2.position.y = t.transform.translation.y;
    collision_pose2.position.z = t.transform.translation.z + 0.774*2;
    addPlanningSceneCollisionObject(id2, dimensions2, collision_pose2, collisionAction::ADD);
    // moveToJointPositions(arm_pick_config, moveGroup::ARM, true);
    arm_move_group_interface->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;
    int c = 0;
    while (!success && c<5){
      success = (arm_move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (success and blocking){
        success = (arm_move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"torso_tray_collision", "torso_main_collision"});
      }
      else if (success and !blocking){
        success = (arm_move_group_interface->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"torso_tray_collision", "torso_main_collision"});
      }
      c++;
    }
    if (c >= 5){
      // planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"torso_tray_collision", "torso_main_collision"});
      ROS_ERROR_STREAM("Could not find plan for pose : " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z);
    }
    addPlanningSceneCollisionObject(id1, std::vector<double>{}, geometry_msgs::Pose(), collisionAction::REMOVE);
    addPlanningSceneCollisionObject(id2, std::vector<double>{}, geometry_msgs::Pose(), collisionAction::REMOVE);
    return success;
}


bool gantryRobot::moveArmToPoseWhileCM(geometry_msgs::Pose target_pose){
    std::cout << "moveArmToPoseWhileCM called!" << std::endl;
    gripperMonitoredFJTStatus = errorCode::SUCCESS;
    moveGroup group = moveGroup::ARM;
    moveit::planning_interface::MoveGroupInterfacePtr interface = arm_move_group_interface;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_client = follow_jt_traj_client;
    // follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
    // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
    bool tempCollisionAdded = true;
    addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::ADD);
    interface->setPoseTarget(target_pose);
    std::cout << "No of joints : " << interface->getJointNames().size() << std::endl;
    bool success, stopped = true;
    
    
    while (stopped){
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        if (tempCollisionAdded)
          std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        else
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
        }
          // success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        else if (tempCollisionAdded){
          std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
          tempCollisionAdded = false;
          addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
          if (success){
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = traj.joint_trajectory;
            fjt_client = follow_jt_traj_client;            
            follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
          }
          else{
            ROS_ERROR_STREAM("Could not find plan for the arm even after removing group collisions!");
            gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
            return false;
          }
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the arm even though group collisions already removed!");
          gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
          return false;
        }
        stopped = false;
        // std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
        // goal.trajectory = traj.joint_trajectory;
        // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
        // stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!fjt_client->getState().isDone()){
          if (!distanceViolated)
            ros::Duration(0.01).sleep();
            // std::cout << "Distance not violated!" << std::endl;
          else{
            fjt_client->cancelGoal();
            sendStopTrajectoryToFJTClient();
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          // ros::Duration(0.01).sleep();
        }
      }
    }

    if (tempCollisionAdded){
      // std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
      // tempCollisionAdded = false;
      addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
    }
    // while (true){
    //   continue;
    // }
    return success;
}


bool gantryRobot::moveArmToPoseWhileCMandGM(geometry_msgs::Pose target_pose){
    std::cout << "moveArmToPoseWhileCMandGM called!" << std::endl;
    moveGroup group = moveGroup::ARM;
    gripperMonitoredFJTStatus = errorCode::SUCCESS;
    moveit::planning_interface::MoveGroupInterfacePtr interface = arm_move_group_interface;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> fjt_client = follow_jt_traj_client;
    // follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
    // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
    bool tempCollisionAdded = true;
    addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::ADD);
    interface->setPoseTarget(target_pose);
    std::cout << "No of joints : " << interface->getJointNames().size() << std::endl;
    bool success, stopped = true;
    
    
    while (stopped){
      if (!objectAttached){
        gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
        break;
      }
      if (!distanceViolated){
        moveit_msgs::RobotTrajectory traj;
        if (tempCollisionAdded)
          std::tie(success, traj)  = getShortestOfNPaths(interface, 20);
        else
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
        if (success){
          control_msgs::FollowJointTrajectoryGoal goal;
          goal.trajectory = traj.joint_trajectory;
          follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
        }
          // success = (interface->execute(traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        else if (tempCollisionAdded){
          std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
          tempCollisionAdded = false;
          addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
          std::tie(success, traj)  = getShortestOfNPaths(interface, 40);
          if (success){
            control_msgs::FollowJointTrajectoryGoal goal;
            goal.trajectory = traj.joint_trajectory;
            fjt_client = follow_jt_traj_client;            
            follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
          }
          else{
            ROS_ERROR_STREAM("Could not find plan for the arm even after removing group collisions!");
            return false;
          }
        }
        else{
          ROS_ERROR_STREAM("Could not find plan for the arm even though group collisions already removed!");
          return false;
        }
        stopped = false;
        // std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
        // goal.trajectory = traj.joint_trajectory;
        // follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
        // stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!fjt_client->getState().isDone()){
          if (objectAttached && !distanceViolated)
            ros::Duration(0.01).sleep();
            // std::cout << "Object attached and distance not violated!" << std::endl;
          else{
            fjt_client->cancelGoal();
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
          // ros::Duration(0.01).sleep();
        }
      }
    }

    if (tempCollisionAdded){
      // std::cout << "Removing planning collisions since plans could not be found otherwise" << std::endl;
      // tempCollisionAdded = false;
      addPlanningCollisionsForGroup(group, std::vector<double>{}, collisionAction::REMOVE);
    }
    // while (true){
    //   continue;
    // }
    return success;
}


std::tuple<double, moveit_msgs::RobotTrajectory> gantryRobot::getCartesianPath(geometry_msgs::Pose initial_pose, geometry_msgs::Pose final_pose, bool collisionCheck){
    std::vector<geometry_msgs::Pose> waypoints = {initial_pose, final_pose};
    // move_group_interface->setMaxVelocityScalingFactor(0.05);
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.02;
    double fraction = arm_move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, collisionCheck);
    std::cout << "Cartesian fraction = " << fraction << ", size of cartesian traj = " << trajectory.joint_trajectory.points.size() << std::endl;
    // move_group_interface->setMaxVelocityScalingFactor(1);
    return std::make_tuple(fraction, trajectory);
}


bool gantryRobot::compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance){
    std::vector<double> cur = move_group_interface->getCurrentJointValues();
    for (int i = 0; i < target.size(); i++){
      if ( abs(cur[i] - target[i]) < tolerance )
        continue;
      else
        return false;
    }
    return true;
}


void gantryRobot::staticPickAndPlaceCallback(const ariac_2021_submission::kittingPickUpStaticObjectGoalConstPtr &goal){
    std::cout << "staticPickAndPlace callback called!" << std::endl;
    ariac_2021_submission::kittingPickUpStaticObjectResult result;
    result.success = PickAndPlace(goal->ID, goal->initial_pose, goal->final_pose);
    std::cout << "Setting spp action result!" << std::endl;
    // pick_as->setSucceeded(result);
    if (result.success)
      pick_as->setSucceeded(result);
    else{
      if (gripperMonitoredFJTStatus == errorCode::GRIPPER_FAILED)
        result.ERROR_CODE = ariac_2021_submission::kittingPickUpStaticObjectFeedback::GRIPPER_FAULT;
      if (gripperMonitoredFJTStatus == errorCode::NO_PLAN_FOUND)
        result.ERROR_CODE = ariac_2021_submission::kittingPickUpStaticObjectFeedback::NO_PLAN_FOUND;
      pick_as->setAborted(result);
    }
    // goal->
}

double gantryRobot::getDifferenceBetweenConfigs(std::vector<double> config1, std::vector<double> config2){
    double diff = -9999;
    if (config1.size() == config2.size()){
      diff = 0;
      for (std::size_t i = 0; i < config1.size(); i++){
        diff += abs(config1[i] - config2[i]);
      }
    }
    return diff;
}

void gantryRobot::inspectionAssemblyCallback(const ariac_2021_submission::inspectionAssemblyGoalConstPtr &goal){
    std::cout << "Inspection assembly request received!" << std::endl;

}

bool gantryRobot::publishToJointPositions(std::vector<double> joint_positions, double time_from_start){
    // jointPositionPublisher.publish()
    trajectory_msgs::JointTrajectory msg;
    /*std::cout << "Current joint names are : " << std::endl;
    std::vector<std::string> joint_names;
    joint_names = move_group_interface->getJointNames();
    std::copy(joint_names.begin(), joint_names.end(), std::ostream_iterator<std::string>(std::cout << ", "));*/
    msg.joint_names.clear();
    // msg.joint_names.push_back("linear_arm_actuator_joint");
    msg.joint_names.push_back("gantry_arm_shoulder_pan_joint");
    msg.joint_names.push_back("gantry_arm_shoulder_lift_joint");
    msg.joint_names.push_back("gantry_arm_elbow_joint");
    msg.joint_names.push_back("gantry_arm_wrist_1_joint");
    msg.joint_names.push_back("gantry_arm_wrist_2_joint");
    msg.joint_names.push_back("gantry_arm_wrist_3_joint");
    msg.points.resize(1);
    msg.points[0].positions = joint_positions;
    msg.points[0].time_from_start = ros::Duration(time_from_start);
    // ros::Rate rate(10);
    bool too_much_time;
    ros::Time start_time = ros::Time::now();
    while (!compareCurrentToTargetJointPositions(joint_positions) && (too_much_time = (ros::Duration(2) > (ros::Time::now() - start_time))) ){
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


bool gantryRobot::PickAndPlace(std::string ID, geometry_msgs::Pose initialPose, geometry_msgs::Pose finalPose, double offset){
    bool success, stopped = false;
    std::cout << "Initial position of object is : " << initialPose.position.y << std::endl;
    ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
    feedback.feedback = feedback.MOVING_TO_OBJECT;
    pick_as->publishFeedback(feedback);
    // _______std::cout << "Moving towards object!" << std::endl;
    std::cout << "Moving towards object!" << std::endl;
    double x_offset = 0;
    double y_offset = -0.5;
    
    moveArmToHomeConfig(true); // Non-blocking might not be preferred due to collision avoidance not being incorporated within the fn itself.
    
    success = moveTorsoToPose(initialPose, x_offset, y_offset, 0, true, false);
    if (success)
      std::cout << "Moved torso to pose using collision monitoring! Success!" << std::endl;
    else
      ROS_ERROR_STREAM("Some error happened inside!");
    next_goal_publisher.publish(initialPose);
    /*while (true){
      if (distanceViolated){
        if (!stopped){
          ROS_ERROR_STREAM("Stopping!");
          torso_move_group_interface->stop();
          stopped = true;
        }
      }
      else if (!stopped){
        if (isPositionDiffLesser(getTransform("torso_base").transform.translation, initialPose.position, x_offset, y_offset, 0.1)){
          std::cout << "Reached!" << std::endl;
          break;
        }
      }
      if (stopped && !distanceViolated){
        ROS_ERROR_STREAM("Starting moveTorsotopose again!");
        success = moveTorsoToPose(initialPose, 0, -0.5, 0, false);
        stopped = false;
      }
      
      
    }
    ros::Duration(1).sleep();*/
    std::cout << "Gantry finished moving to object" << std::endl;
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
    ros::Subscriber gripper_sub = _nh->subscribe("arm/gripper/state", 5, &gantryRobot::gripperStateCb, this);
    feedback.feedback = feedback.PREGRASP;
    pick_as->publishFeedback(feedback);
    tf2::Quaternion pre_grasp_orientation;
    pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z),0,0)});
    // else
    //   pre_grasp_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(initialPose.orientation).z + M_PI),0,0)});  // rotation mult (0,pi/2,0) and (-roll,0,0)
    geometry_msgs::Pose pre_grasp_pose, grasp_pose_estimate;
    pre_grasp_pose.orientation = tf2::toMsg(pre_grasp_orientation);
    pre_grasp_pose.position = initialPose.position;
    pre_grasp_pose.position.z += offset;
    // success = moveToJointPositions(arm_pick_config, moveGroup::ARM, true);
    success = moveArmToPoseWhileCM(pre_grasp_pose);
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
        std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(arm_move_group_interface->getCurrentPose().pose, pre_grasp_pose);
        if (approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0){
          moveit_msgs::RobotTrajectory corrected_traj;
          corrected_traj = cartesian_traj_approach;
          // for (auto i : corrected_traj.joint_trajectory.points){
          //   i.positions.clear();
          //   i.time_from_start.
          // }
          corrected_traj.joint_trajectory.points.clear();
          // trajectory_msgs::JointTrajectoryPoint p;
          int count = 0;
          double prev_time = -99999;
          for (auto i : cartesian_traj_approach.joint_trajectory.points){
            if (count > 0){
              if (i.time_from_start.toSec() > prev_time){
                corrected_traj.joint_trajectory.points.push_back(i);
                prev_time = i.time_from_start.toSec();
              }
            }
            count++;
          }
          // robot_trajectory::RobotTrajectory rt(arm_move_group_interface->getRobotModel(), joint_model_group);
          // rt.setRobotTrajectoryMsg(*arm_move_group_interface->getCurrentState(), cartesian_traj_approach);
          // trajectory_processing::IterativeParabolicTimeParameterization iptp;
          // iptp.computeTimeStamps(rt, 0.1, 0.1);
          // moveit_msgs::RobotTrajectory modified_traj;
          // rt.getRobotTrajectoryMsg(modified_traj);
          ros::Duration(0.5).sleep();
          success = (arm_move_group_interface->execute(corrected_traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }
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
        collision_pose.position.z = Transform.transform.translation.z - 0.07;
        std::string id = "temp_collision";
        dimensions[0] = 0.15;
        dimensions[1] = 0.15;
        dimensions[2] = 0.125;
        collision_pose.orientation.w = 1;
        addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
        arm_move_group_interface->attachObject("temp_collision");
        // if (approach_fraction < 0.99 || !success){
        //   std::cout << "Cartesian retract did not work! Waiting for a sec and then performing moveArmtoPose" << std::endl;
        //   ros::Duration(1).sleep();
        //   success = moveArmToPose(pre_grasp_pose, true);
        // }
      }
      count++;
    }
    // Start a callback to see if object remains attached to the gripper

    if (pick_as->isPreemptRequested() || !_nh->ok()){
        ROS_INFO("Static pick and place preempted");
        pick_as->setPreempted();
        return false;
    }

    // std::vector<double> curr_torso_jts = torso_move_group_interface->getCurrentJointValues();
    pre_grasp_pose.position.z += 0.1;
    pre_grasp_pose.position.y += 0.05;
    std::tie(approach_fraction, cartesian_traj_approach) = getCartesianPath(arm_move_group_interface->getCurrentPose().pose, pre_grasp_pose);
    if (approach_fraction > 0.99 && cartesian_traj_approach.joint_trajectory.points.size()>0){
      moveit_msgs::RobotTrajectory corrected_traj;
      corrected_traj = cartesian_traj_approach;
      // for (auto i : corrected_traj.joint_trajectory.points){
      //   i.positions.clear();
      //   i.time_from_start.
      // }
      corrected_traj.joint_trajectory.points.clear();
      // trajectory_msgs::JointTrajectoryPoint p;
      int count = 0;
      double prev_time = -99999;
      for (auto i : cartesian_traj_approach.joint_trajectory.points){
        if (count > 0){
          if (i.time_from_start.toSec() > prev_time){
            corrected_traj.joint_trajectory.points.push_back(i);
            prev_time = i.time_from_start.toSec();
          }
        }
        count++;
      }
      // robot_trajectory::RobotTrajectory rt(arm_move_group_interface->getRobotModel(), joint_model_group);
      // rt.setRobotTrajectoryMsg(*arm_move_group_interface->getCurrentState(), cartesian_traj_approach);
      // trajectory_processing::IterativeParabolicTimeParameterization iptp;
      // iptp.computeTimeStamps(rt, 0.1, 0.1);
      // moveit_msgs::RobotTrajectory modified_traj;
      // rt.getRobotTrajectoryMsg(modified_traj);
      // ros::Duration(0.5).sleep();
      success = (arm_move_group_interface->execute(corrected_traj) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    std::vector<double> des_arm_jts = arm_move_group_interface->getCurrentJointValues();
    // des_arm_jts[0] += 1.5;
    des_arm_jts[1] -= M_PI/2;  // outwards
    des_arm_jts[2] += 0.8;  // inwards
    // curr_arm_
    if (!success){
      ROS_ERROR_STREAM("Could not retract!");
    }
    moveToJointPositions(des_arm_jts, moveGroup::ARM, true);
    moveArmToHomeConfig(true);
    feedback.feedback = feedback.MOVE_TO_GOAL;
    pick_as->publishFeedback(feedback);
    std::cout << "Pickup finished!" << std::endl;

    //  -------------- PLACE -------------------
    // success = moveTorsoToPose(finalPose, 0, -0.5, true);
    // x_base = getTransform("base_link").transform.translation.x;
    // bool finalFront = (finalPose.position.x > x_base);
    // moveTowardsObject(finalPose, finalFront, true);
    next_goal_publisher.publish(finalPose);
    if (!moveTorsoToPose(finalPose, x_offset, y_offset, 0, true, true)){
      objectAttached = true;
      gripper_sub.shutdown();
      return false;
    }
    std::cout << "Woohoo! FINISHED moving towards object while grasping successfully!" << std::endl;
    /*success = moveTorsoToPose(finalPose, x_offset, y_offset, 0, false);
    while (true){
      if (distanceViolated){
        if (!stopped){
          ROS_ERROR_STREAM("Stopping!");
          torso_move_group_interface->stop();
          stopped = true;
        }
      }
      else if (!stopped){
        if (isPositionDiffLesser(getTransform("torso_base").transform.translation, finalPose.position, x_offset, y_offset, 0.1)){
          std::cout << "Reached!" << std::endl;
          break;
        }
      }
      if (stopped && !distanceViolated){
        ROS_ERROR_STREAM("Starting moveTorsotopose again!");
        success = moveTorsoToPose(finalPose, 0, -0.5, 0, false);
        stopped = false;
      }
      
      
    }*/
    // ros::Duration(1).sleep();
    feedback.feedback = feedback.PLACE;
    pick_as->publishFeedback(feedback);
    std::cout << "Attempting to move to pre_place_pose!" << std::endl;
    tf2::Quaternion pre_place_orientation;
    pre_place_orientation = getNetRotation(std::vector<RPY>{RPY(0,M_PI_2,0),RPY(-(rpyFromQuat(finalPose.orientation).z),0,0)});
    geometry_msgs::Pose pre_place_pose, place_pose;
    pre_place_pose.orientation = tf2::toMsg(pre_place_orientation);
    pre_place_pose.position = finalPose.position;
    // success = moveTowardsObject(finalPose, finalFront, true);
    pre_place_pose.position.z += (offset - placeOffsetZ + 0.01);

    planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"temp_collision"});
    
    if (!moveArmToPoseWhileCMandGM(pre_place_pose)){
      ROS_ERROR_STREAM("Failed to move arm to pose while both CM and GM");
      return false;
    }
    
    gripper_srv.request.enable = false;

    if (success)
      if (!gripperControl.call(gripper_srv)){
        ROS_ERROR_STREAM("Unable to turn off Gripper!");
        return false;
      }
    std::cout << "Dropped the part! Moving to home configuration!" << std::endl;
    success = moveArmToHomeConfig(true);

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
    // planning_scene_interface.removeCollisionObjects(std::vector<std::string>{"temp_collision"});
    gripper_sub.shutdown();
    return success;
}






void gantryRobot::probe(moveit_msgs::RobotTrajectory trajectory, double vel_scale, double acc_scale){
    // finishedProbe = false;
    std::cout << "Probe called" << std::endl;
    boost::shared_ptr<const sensor_msgs::JointState> msg = ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states", *_nh, ros::Duration(5));
    wrist_1_joint_index = std::distance(msg->name.begin(), std::find(msg->name.begin(), msg->name.end(), "gantry_arm_wrist_1_joint"));
    std::cout << "Index of wrist 1 joint is : " << wrist_1_joint_index << std::endl;
    robot_trajectory::RobotTrajectory rt(arm_move_group_interface->getRobotModel(), joint_model_group);
    rt.setRobotTrajectoryMsg(*arm_move_group_interface->getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, vel_scale, acc_scale);
    // moveit_msgs::RobotTrajectory current_traj;
    // current_traj.joint_trajectory.header = trajectory.joint_trajectory.header;
    // current_traj.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    // wrist_1_joint_index = std::distance(trajectory.joint_trajectory.joint_names.begin(), std::find(trajectory.joint_trajectory.joint_names.begin(), trajectory.joint_trajectory.joint_names.end(), "wrist_1_joint"));
    // double desired_effort;
    joint_states_sub = _nh->subscribe<sensor_msgs::JointState>("joint_states", 10, &gantryRobot::jointStatesCallback, this);
    control_msgs::FollowJointTrajectoryGoal goal;
    // goal.goal_time_tolerance = ros::Duration(10);
    moveit_msgs::RobotTrajectory modified_traj;
    rt.getRobotTrajectoryMsg(modified_traj);
    goal.trajectory = modified_traj.joint_trajectory;
    // joint_states_sub = _nh->subscribe<sensor_msgs::JointState>("joint_states", 10, &gantryRobot::jointStatesCallback, this);
    // control_msgs::FollowJointTrajectoryGoal goal;
    // goal.trajectory = trajectory.joint_trajectory;
    // for (auto i : goal.trajectory.points){
    //   i.time_from_start = ros::Duration(i.time_from_start.toSec()*25);
    // }
    // ros::Rate rate(100);
    finishedProbe = false;
    follow_jt_traj_client->sendGoal(goal, boost::bind(&gantryRobot::fjtDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtActiveCb, this), boost::bind(&gantryRobot::fjtFeedbackCb, this, _1));
    ros::Time start_time = ros::Time::now();
    // follow_jt_traj_client->waitForResult();
    while (!finishedProbe){
      ros::spinOnce();
      if ((ros::Time::now() - start_time).toSec() > 15){
        ROS_ERROR_STREAM("Taking more than 15 secs!");
        break;
      }
    }

    std::cout << "Sending empty traj to lift a bit!" << std::endl;
    joint_states_sub.shutdown();
    sendEmptyTrajectoryToFJTClient();
    std::vector<double> currentConfig = arm_move_group_interface->getCurrentJointValues();
    currentConfig[2] += 0.1;
    publishToJointPositions(currentConfig, 0.2);
    // follow_jt_traj_client->waitForResult();

    // while(!finishedProbe){
    //   ros::spinOnce();
    // }
    std::cout << "FJT either cancelled or succeeded inside probe! LOL!" << std::endl;
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


errorCode gantryRobot::moveTowardsObjectWhileGripperMonitoring(geometry_msgs::Pose pose){
    gripperMonitoredFJTStatus = errorCode::SUCCESS;
    std::cout << "Moving towards object while monitoring" << std::endl;
    std::vector<double> config /*= torso_move_group_interface->getCurrentJointValues()*/;
    config.resize(3, 0);
    config[0] = pose.position.x + 0 + 2;
    config[1] = (-pose.position.y) + 0.5;
    config[2] = 0;
    // bool success = moveToJointPositions(config, moveGroup::FULL, true);

    torso_move_group_interface->setJointValueTarget(config);
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // torso_move_group_interface->plan(plan);
    moveit_msgs::RobotTrajectory traj;
    bool success, stopped = true;
    std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
    control_msgs::FollowJointTrajectoryGoal goal;
    while (stopped){
      if (!distanceViolated){
        std::tie(success, traj) = getShortestOfNPaths(torso_move_group_interface, 20);
        goal.trajectory = traj.joint_trajectory;
        follow_jt_traj_torso_client->sendGoal(goal, boost::bind(&gantryRobot::fjtTorsoMonitorDoneCb, this, _1, _2), boost::bind(&gantryRobot::fjtTorsoMonitorActiveCb, this), boost::bind(&gantryRobot::fjtTorsoMonitorFeedbackCb, this, _1));
        stopped = false;
      }
      // success = follow_jt_traj_torso_client->waitForResult();
      if (success){
        while (!follow_jt_traj_torso_client->getState().isDone()){
          if (objectAttached && !distanceViolated)
            ros::Duration(0.01).sleep();
            // std::cout << "Object : Attached!" << std::endl;
          else{
            follow_jt_traj_torso_client->cancelGoal();
            sendEmptyTrajectoryToFJTTorsoClient();
            if (!objectAttached){
              ROS_ERROR_STREAM("Gripper failed while performing gripper monitored motion");
              // ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
              // feedback.feedback = feedback.GRIPPER_FAULT;
              // pick_as->publishFeedback(feedback);
              gripperMonitoredFJTStatus = errorCode::GRIPPER_FAILED;
              // follow_jt_traj_torso_client->cancelGoal();
            }
            if (distanceViolated){
              stopped = true;
            }
            break;
          }
          // ros::Duration(0.01).sleep();
        }
      }
    }
    // if (!gripperMonitoredFJTStatus){
    //   std::cout << "YEEEHAAWWWWW!" << std::endl;
    //   success = true;
    // }
    // else{
    //   std::cout << "Nuuuuu. Apparently FJT failed." << std::endl;
    //   success = false;
    //   gripperMonitoredFJTStatus = false;
    // }
    
    if (!success)
      gripperMonitoredFJTStatus = errorCode::NO_PLAN_FOUND;
    return gripperMonitoredFJTStatus;
}

void gantryRobot::sendEmptyTrajectoryToFJTClient(){
    control_msgs::FollowJointTrajectoryGoal goal;
    /*goal.trajectory.joint_names = arm_move_group_interface->getJointNames();
    goal.trajectory.points.resize(2);
    goal.trajectory.points[0].positions = arm_move_group_interface->getCurrentJointValues();
    goal.trajectory.points[0].positions[2] -= 0.001;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    goal.trajectory.points[0].positions[4] += 0.008;   // prev 0.03
    goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.01;
    goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.2);
    follow_jt_traj_client->sendGoal(goal);*/
    goal.trajectory.joint_names = arm_move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = arm_move_group_interface->getCurrentJointValues();
    // goal.trajectory.points[0].positions[3] += 0.0005;
    // goal.trajectory.points[0].positions[4] += 0.03;   // prev 0.03
    // goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    // goal.trajectory.points[0].effort[4] = 3;
    // // goal.trajectory.points[0].positions[2] -= 0.01;
    goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.3);

    // goal.trajectory.points[1].positions[2] += 0.1;
    // // goal.trajectory.points[1].positions[1] -= 0.1;
    // // goal.trajectory.points[1].positions[3] +=
    // // goal.trajectory.points[1].positions[2] += 0.01;
    // goal.trajectory.points[1].time_from_start = ros::Duration(0.5);
    follow_jt_traj_client->sendGoal(goal);
    follow_jt_traj_client->waitForResult(ros::Duration(1));
}


void gantryRobot::sendStopTrajectoryToFJTClient(){
    control_msgs::FollowJointTrajectoryGoal goal;
    /*goal.trajectory.joint_names = arm_move_group_interface->getJointNames();
    goal.trajectory.points.resize(2);
    goal.trajectory.points[0].positions = arm_move_group_interface->getCurrentJointValues();
    goal.trajectory.points[0].positions[2] -= 0.001;
    // goal.trajectory.points[0].positions[3] += 0.0005;
    goal.trajectory.points[0].positions[4] += 0.008;   // prev 0.03
    goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.01;
    goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.2);
    follow_jt_traj_client->sendGoal(goal);*/
    goal.trajectory.joint_names = arm_move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = arm_move_group_interface->getCurrentJointValues();
    goal.trajectory.points[0].time_from_start = ros::Duration(0.01);
    // goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    follow_jt_traj_client->sendGoal(goal);
}


void gantryRobot::sendEmptyTrajectoryToFJTTorsoClient(){
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = torso_move_group_interface->getJointNames();
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = torso_move_group_interface->getCurrentJointValues();
    // goal.trajectory.points[0].positions[2] -= 0.001;
    // // goal.trajectory.points[0].positions[3] += 0.0005;
    // goal.trajectory.points[0].positions[4] += 0.008;   // prev 0.03
    // goal.trajectory.points[0].effort.resize(goal.trajectory.joint_names.size());
    // goal.trajectory.points[0].effort[4] = 3;
    // goal.trajectory.points[0].positions[2] -= 0.01;
    // goal.trajectory.points[0].velocities.resize(goal.trajectory.joint_names.size(), 0);
    goal.trajectory.points[0].time_from_start = ros::Duration(0.01);
    follow_jt_traj_client->sendGoal(goal);
}


std::tuple<bool, moveit_msgs::RobotTrajectory> gantryRobot::getShortestOfNPaths(moveit::planning_interface::MoveGroupInterfacePtr interface, int count){
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

moveit_msgs::RobotTrajectory gantryRobot::getShortestPath(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans){
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






void gantryRobot::fjtFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // std::cout << "Diff between desired and actual joint efforts = " << abs(feedback->error.effort[wrist_1_joint_index]) << std::endl;
    
}

void gantryRobot::fjtDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    std::cout << "FJT finished with state : " << state.toString().c_str() << std::endl;
    std::cout << "Result is : " << (result->error_code == result->SUCCESSFUL) << std::endl;

}

void gantryRobot::fjtActiveCb(){
    std::cout << "FJT Goal just went active!" << std::endl;
}

void gantryRobot::fjtMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // std::cout << "Diff between desired and actual joint efforts = " << abs(feedback->error.effort[wrist_1_joint_index]) << std::endl;
    if (objectAttached)
      std::cout << "FJT Monitor feedback : Attached!" << std::endl;
    else{
      follow_jt_traj_client->cancelGoal();
      ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
      feedback.feedback = feedback.GRIPPER_FAULT;
      pick_as->publishFeedback(feedback);
      // gripperMonitoredFJTStatus = errorCode::SUCCESS;
      follow_jt_traj_client->cancelGoal();
      sendEmptyTrajectoryToFJTClient();
    }
}

void gantryRobot::fjtMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    std::cout << "FJT Monitor finished with state : " << state.toString().c_str() << std::endl;
    std::cout << "Result is : " << (result->error_code == result->SUCCESSFUL) << std::endl;

}

void gantryRobot::fjtTorsoMonitorActiveCb(){
    std::cout << "FJT Monitor Goal just went active!" << std::endl;
}

void gantryRobot::fjtTorsoMonitorFeedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& feedback){
    // std::cout << "Diff between desired and actual joint efforts = " << abs(feedback->error.effort[wrist_1_joint_index]) << std::endl;
    // if (objectAttached)
    //   std::cout << "FJT Monitor feedback : Attached!" << std::endl;
    // else{
    //   follow_jt_traj_torso_client->cancelGoal();
    //   ariac_2021_submission::kittingPickUpStaticObjectFeedback feedback;
    //   feedback.feedback = feedback.GRIPPER_FAULT;
    //   pick_as->publishFeedback(feedback);
    //   gripperMonitoredFJTFailed = true;
    //   follow_jt_traj_torso_client->cancelGoal();
    //   sendEmptyTrajectoryToFJTTorsoClient();
    // }
}

void gantryRobot::fjtTorsoMonitorDoneCb(const actionlib::SimpleClientGoalState& state, const control_msgs::FollowJointTrajectoryResultConstPtr& result){
    std::cout << "FJT Monitor finished with state : " << state.toString().c_str() << std::endl;
    std::cout << "Result is : " << (result->error_code == result->SUCCESSFUL) << std::endl;

}

void gantryRobot::fjtMonitorActiveCb(){
    std::cout << "FJT Monitor Goal just went active!" << std::endl;
}

void gantryRobot::gripperStateCb(const nist_gear::VacuumGripperStateConstPtr& state){
    objectAttached = state->attached;
    // std::cout << "Attached = " << objectAttached << std::endl;
}

void gantryRobot::jointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg){
    // wrist_1_joint_index
    double effort = joint_state_msg->effort[wrist_1_joint_index];
    std::cout << " wrist_1_joint effort : " << effort /*<< ", finishedProbe : " << finishedProbe */<< std::endl;
    if (abs(effort) > 2.5){
      std::cout << "Cancelling current goal!" << std::endl;
      follow_jt_traj_client->cancelGoal();
      std::cout << "Goal cancelled!" << std::endl;
      // sendEmptyTrajectoryToFJTClient();
      finishedProbe = true;
    }
}

void gantryRobot::track_kitting_timer_callback(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped t = getTransform("torso_base", "base_link");
    if (abs(t.transform.translation.x) < 1.5 && abs(t.transform.translation.y) < 1.5 ){
      distanceViolated = true;
    }
    else
      distanceViolated = false;

    // std::cout << "Diff btw kitting and gantry : ( " << t.transform.translation.x << ", " << t.transform.translation.y << " )" << std::endl; 
}

bool gantryRobot::addCollisionSpheres(){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface->getPlanningFrame();
    collision_object.id = "temp_spheres";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.SPHERE;
    for (std::vector<double>::const_iterator it = sphereRadii.begin(); it != sphereRadii.end(); it++){
      if (it != sphereRadii.end()-2){
        primitive.dimensions = std::vector<double>{*it};
        collision_object.primitives.push_back(primitive);
      }
      else{
        std::cout << "Omitted radius is : " << *it << std::endl;
      }
    }
    // primitive.dimensions = std::vector<double>(sphereRadii.begin(), sphereRadii.end()-1);
    // primitive.dimensions = sphereRadii;
    // ROS_ERROR_STREAM("There are " << collision_object.primitives.size() << " primitives given");
    // std::vector<geometry_msgs::Pose> poses;
    // poses.resize(primitive.dimensions.size());
    // torso_base can be used for {x,y}, torso_main for yaw.
    std::vector<geometry_msgs::TransformStamped> transforms{getTransform("torso_tray"), getTransform("gantry_arm_base_link"), 
      getTransform("gantry_arm_forearm_link"), getTransform("gantry_arm_wrist_1_link"),
      getTransform("gantry_arm_vacuum_gripper_link")};
    geometry_msgs::Point t;
    t.x = transforms[0].transform.translation.x;
    t.y = transforms[0].transform.translation.y;
    t.z = transforms[0].transform.translation.z + 0.65;
    collision_object.primitive_poses.push_back(pointToPose(t));
    t.x = avg(std::vector<double>{transforms[1].transform.translation.x, transforms[2].transform.translation.x});
    t.y = avg(std::vector<double>{transforms[1].transform.translation.y, transforms[2].transform.translation.y});
    t.z = avg(std::vector<double>{transforms[1].transform.translation.z, transforms[2].transform.translation.z});
    collision_object.primitive_poses.push_back(pointToPose(t));
    t.x = avg(std::vector<double>{transforms[2].transform.translation.x, transforms[3].transform.translation.x});
    t.y = avg(std::vector<double>{transforms[2].transform.translation.y, transforms[3].transform.translation.y});
    t.z = avg(std::vector<double>{transforms[2].transform.translation.z, transforms[3].transform.translation.z});
    collision_object.primitive_poses.push_back(pointToPose(t));
    t.x = avg(std::vector<double>{transforms[3].transform.translation.x, transforms[4].transform.translation.x});
    t.y = avg(std::vector<double>{transforms[3].transform.translation.y, transforms[4].transform.translation.y});
    t.z = avg(std::vector<double>{transforms[3].transform.translation.z, transforms[4].transform.translation.z});
    // collision_object.primitive_poses.push_back(pointToPose(t));
    
    geometry_msgs::TransformStamped t1;
    t1.header.frame_id = "gantry_arm_vacuum_gripper_link";
    t1.transform.translation.x = t1.transform.translation.y = 0;
    t1.transform.translation.z = -0.135;
    t1.transform.rotation = quatFromRPY(0,0,0);
    tf2::doTransform(t1, t1, transforms[4]);
    
    collision_object.primitive_poses.push_back(vectorToPose(t1.transform.translation));
    

    // if (Action == collisionAction::ADD)
    collision_object.operation = collision_object.ADD;
    // else if (Action == collisionAction::REMOVE)
    //   collision_object.operation = collision_object.REMOVE;
    // else if (Action == collisionAction::MOVE)
    //   collision_object.operation = collision_object.MOVE;
    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    collisionObjects.push_back(collision_object);
    if ( planning_scene_interface.getObjects(std::vector<std::string>{collision_object.id}).size() > 0){
      moveit_msgs::CollisionObject col = collision_object;
      col.operation = col.REMOVE;
      std::vector<moveit_msgs::CollisionObject> c1{col};
      planning_scene_interface.applyCollisionObjects(c1);
    }
    std::cout << "Adding spheres into the scene" << std::endl;
    return planning_scene_interface.applyCollisionObjects(collisionObjects);
}


bool gantryRobot::addPlanningCollisionsForGroup(moveGroup group, std::vector<double> j_values, collisionAction action){
    std::vector<std::string> ids;
    ids.resize(4);
    std::vector<double> dimensions;
    dimensions.resize(3);
    geometry_msgs::Pose collision_pose;
    bool success = true;
    switch (group){
      case moveGroup::ARM :{
        ids.resize(2);
        ids[0] = "torso_tray_collision";
        dimensions[0] = 0.88;
        dimensions[1] = 0.77;
        dimensions[2] = 0.0192*2;
        collision_pose.orientation.w = 1;
        geometry_msgs::TransformStamped t = getTransform("torso_tray");
        collision_pose.position.x = t.transform.translation.x;
        collision_pose.position.y = t.transform.translation.y;
        collision_pose.position.z = t.transform.translation.z - 0.0192;
        success = success && addPlanningSceneCollisionObject(ids[0], dimensions, collision_pose, action);
      

      
        ids[1] = "torso_main_collision";  // 0.921, 0.698, 0.921. Transform of {0,0,0.774}
        dimensions[0] = 0.921;
        dimensions[1] = 0.698;
        dimensions[2] = 0.921;
        collision_pose.orientation.w = 1;
        t = getTransform("torso_main");
        collision_pose.position.x = t.transform.translation.x;
        collision_pose.position.y = t.transform.translation.y;
        collision_pose.position.z = t.transform.translation.z + 0.774*2;
        success = success && addPlanningSceneCollisionObject(ids[1], dimensions, collision_pose, action);
        break;
      }
      case moveGroup::TORSO :{
        std::vector<double> torso_jvs = move_group_interface->getCurrentJointValues();
        // Adding conveyor collisions
        ids[0] = "torso_planning_collision0";
        dimensions[0] = 0.01;
        dimensions[1] = 20;
        dimensions[2] = 2.8;
        collision_pose.orientation = quatFromRPY(0,0,0);
        if (j_values.size() != 3){
          j_values.resize(3, 0);
        }
        collision_pose.position.x = min({j_values[0]-2, torso_jvs[0]-2}) - 2;
        collision_pose.position.y = 0;
        collision_pose.position.z = dimensions[2]/2;
        success = success && addPlanningSceneCollisionObject(ids[0], dimensions, collision_pose, action);
        collision_pose.position.x = max({j_values[0]-2, torso_jvs[0]-2}) + 2;
        ids[1] = "torso_planning_collision1";
        success = success && addPlanningSceneCollisionObject(ids[1], dimensions, collision_pose, action);
        ids[2] = "torso_planning_collision2";
        collision_pose.orientation = quatFromRPY(0,0,M_PI_2);
        collision_pose.position.x = 0;
        collision_pose.position.y = min({-j_values[1], -torso_jvs[1]}) - 2;
        success = success && addPlanningSceneCollisionObject(ids[2], dimensions, collision_pose, action);
        ids[3] = "torso_planning_collision3";
        collision_pose.position.x = 0;
        collision_pose.position.y = max({-j_values[1], -torso_jvs[1]}) + 2;
        success = success && addPlanningSceneCollisionObject(ids[3], dimensions, collision_pose, action);
        break;
      }
    }
    return success;
}


bool gantryRobot::addInitialCollisionObjects(){
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

    // AGV poles
    {
      // kit_tray_1 - kit_tray_4
      std::string kit_prefix = "kit_tray_";
      dimensions[0] = 0.253;
      dimensions[1] = 0.253;
      dimensions[2] = 1;
      for (int i=1; i<=4; i++){
        id = kit_prefix + std::to_string(i) + "_pole_collision";
        geometry_msgs::TransformStamped transformStamped = getTransform(kit_prefix + std::to_string(i));
        std::cout << "EEEEEE " << transformStamped.transform.translation.x << ", " << transformStamped.transform.translation.y << ", " << transformStamped.transform.translation.z << std::endl;
        collision_pose.orientation = quatFromRPY(0,0,0);
        collision_pose.position.x = transformStamped.transform.translation.x - 0.15 - 0.28156;
        collision_pose.position.y = transformStamped.transform.translation.y;
        collision_pose.position.z = 0.9015;
        addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
      }
    }

    /*{   // Need to add these collisions only in the case of moveArmToPose momentarily planning
      id = "torso_tray_collision";
      dimensions[0] = 0.88;
      dimensions[1] = 0.77;
      dimensions[2] = 0.0192*2;
      collision_pose.orientation.w = 1;
      geometry_msgs::TransformStamped t = getTransform("torso_tray");
      collision_pose.position.x = t.transform.translation.x;
      collision_pose.position.y = t.transform.translation.y;
      collision_pose.position.z = t.transform.translation.z - 0.0192;
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }

    {
      id = "torso_main_collision";  // 0.921, 0.698, 0.921. Transform of {0,0,0.774}
      dimensions[0] = 0.921;
      dimensions[1] = 0.698;
      dimensions[2] = 0.921;
      collision_pose.orientation.w = 1;
      geometry_msgs::TransformStamped t = getTransform("torso_main");
      collision_pose.position.x = t.transform.translation.x;
      collision_pose.position.y = t.transform.translation.y;
      collision_pose.position.z = t.transform.translation.z + 0.774*2;
      addPlanningSceneCollisionObject(id, dimensions, collision_pose, collisionAction::ADD);
    }*/

    // NEED TO ADD AGVS?

    std::cout << "Finished adding the objects" << std::endl;

}