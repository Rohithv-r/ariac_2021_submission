#include <iostream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/Product.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ariac_2021_submission/addPlanningSceneCollision.h>
#include <ariac_2021_submission/pickupStaticObjectAction.h>

tf2_ros::Buffer tfBuffer;
boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
ros::ServiceClient addPlanningSceneCollisionKittingClient;
ros::CallbackQueue action_queue;

void start_competition(const ros::NodeHandlePtr& node) {
    // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
    ros::ServiceClient start_client =
      node->serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    if (!start_client.exists()) {
      ROS_INFO("Waiting for the competition to be ready...");
      start_client.waitForExistence();
      ROS_INFO("Competition is now ready.");
    }
    ROS_INFO("Requesting competition start...");
    std_srvs::Trigger srv;
    start_client.call(srv);
    if (!srv.response.success) {
      ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
    } else {
      ROS_INFO("Competition started!");
    }
}

bool positionEquality(geometry_msgs::Pose p1, geometry_msgs::Pose p2, double tolerance = 0.02){
    if ( abs(p1.position.x-p2.position.x)<=tolerance && 
          abs(p1.position.y-p2.position.y)<=tolerance && 
            abs(p1.position.z-p2.position.z)<=tolerance ) {
              // std::cout << "Positions not equal" << std::endl;
              return true;
            }
    return false;
}

bool isPositionDiffLesser(geometry_msgs::Vector3 t1, geometry_msgs::Point t2, double diff=0.3){
    if( (abs(t1.x - t2.x) < diff) && (abs(t1.y - t2.y) < diff) ){
      return true;
    }
    // std::cout << abs(t1.x - t2.x) << ", " << abs(t1.y - t2.y) << std::endl;
    return false;
}

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

void copyPoseToTransform(geometry_msgs::Pose p, geometry_msgs::Transform& t){
    t.translation.x = p.position.x;
    t.translation.y = p.position.y;
    t.translation.z = p.position.z;
    t.rotation = p.orientation;
}

nist_gear::Model getModelFromProduct(nist_gear::Product p) {
    nist_gear::Model m;
    m.type = p.type;
    m.pose = p.pose;
    return m;
}


// ---------------------------- AVAILABLEPARTS ---------------------------------------------


struct availablePart{
  
    std::string type;
    std::string type_id;
    bool req_for_kitting = false;
    bool inAction = false;
    // bool conveyor;
    geometry_msgs::PoseStamped pose;
    // geometry_msgs::TransformStamped estimated_pose;
    ros::Time lastPoseUpdateTime;
    double conveyorVel = 0.2;
    int bin_id = 0;

    availablePart() {
      lastPoseUpdateTime = ros::Time::now();
    }
    availablePart(std::string in_type, std::string ID, geometry_msgs::PoseStamped in_pose) 
      : type(in_type), type_id(ID), pose(in_pose) {

        lastPoseUpdateTime = ros::Time::now();      
    }
    // std::string getType() { return type; }
    // bool inAction() { return inAction; }
    // void setInAction(bool act) { inAction = act; }
    // bool onConveyor() { return conveyor; }
    // void setConveyor(bool conv) { conveyor = conv; }
    // void setBin(int bin) { bin_id = bin; }
    void estimateAndUpdateTransform();
    bool hasConveyorPartExceededLimit(double threshold);

};

void availablePart::estimateAndUpdateTransform(){
    ros::Duration diff = ros::Time::now() - lastPoseUpdateTime;
    pose.pose.position.y -= conveyorVel*(diff.sec+diff.nsec*pow(10,-9));
    lastPoseUpdateTime = ros::Time::now();
}

bool availablePart::hasConveyorPartExceededLimit(double threshold){
    if (pose.pose.position.y > threshold)
      return true;
    return false;
}

class availablePartsManager{
  private:
    std::vector<availablePart> availableParts;
    std::vector<availablePart> conveyorParts;
    std::map<std::string,int> type_count;
    std::map<std::string,int> type_id_prev;
    void addTypeCount(std::string type, int count);
    void addTypeIdPrev(std::string type, int id);

  public:
    availablePartsManager() {
      
    }
    // void update();
    // availablePart mostRecentPart;
    void updateTransforms();
    std::vector<availablePart> getConveyorPartsExceedingLimit(double threshold = 1);
    int getTypeCount(std::string type);
    int getNonRequiredTypeCount(std::string type);
    availablePart getNonRequiredPart(std::string type, bool markAsRequired);  // Always call getNonRequiredTypeCount first
    int getTypeIdPrev(std::string type);
    std::vector<availablePart> getAvailableParts() { return availableParts;}
    bool addPartFromLogicalCamera(nist_gear::Model m, std::string camera_frame);
    void updateKittableParts(std::vector<std::string>);
    void setInAction(std::string typeID, bool inAction = true);
    availablePart getStaticKittablePart();
    std::vector<availablePart> getStaticKittableParts();
};

int availablePartsManager::getNonRequiredTypeCount(std::string type){
    int count = 0;
    for (auto i : availableParts){
      if (!i.req_for_kitting && i.type == type){
        count++;
      }
    }
    return count;
}

void availablePartsManager::setInAction(std::string typeID, bool inAction){
    std::vector<availablePart>::iterator it;
    if ( ( it = std::find_if(availableParts.begin(), availableParts.end(), [typeID] (availablePart p) { return (p.type_id == typeID); }) ) != availableParts.end()){
      it->inAction = inAction;
    }
}

availablePart availablePartsManager::getNonRequiredPart(std::string type, bool markAsRequired){
    for (auto i : availableParts){
      if (!i.req_for_kitting){
        if (i.type == type){
          i.req_for_kitting = true;
          return i;
        }
      }
    }
}

void availablePartsManager::updateKittableParts(std::vector<std::string> requiredTypes){
    for (auto i : availableParts){
      if (!i.req_for_kitting){
        if ( (std::find(requiredTypes.begin(), requiredTypes.end(), i.type) != requiredTypes.end()))
          i.req_for_kitting = true;
      }
    }
}

int availablePartsManager::getTypeCount(std::string type){
    std::map<std::string,int>::iterator it = type_count.find(type);
    if (it != type_count.end()){
      return it->second;
    }
    else
      return 0;
}

int availablePartsManager::getTypeIdPrev(std::string type){
    std::map<std::string,int>::iterator it = type_id_prev.find(type);
    if (it != type_id_prev.end()){
      return it->second;
    }
    else
      return 0;
}

availablePart availablePartsManager::getStaticKittablePart(){
    for (auto i : availableParts){
      if (i.req_for_kitting)
        return i;
    }
}

std::vector<availablePart> availablePartsManager::getStaticKittableParts(){
    std::vector<availablePart> parts;
    for (auto i : availableParts){
      if (i.req_for_kitting)
        parts.push_back(i);
    }
    return parts;
}

void availablePartsManager::updateTransforms() {
    for (std::vector<availablePart>::iterator it = conveyorParts.begin(); it!=conveyorParts.end(); it++){
      it->estimateAndUpdateTransform();
    }
}

std::vector<availablePart> availablePartsManager::getConveyorPartsExceedingLimit(double threshold){
    std::vector<availablePart> parts;
    for (std::vector<availablePart>::iterator it = conveyorParts.begin(); it!=conveyorParts.end(); it++){
      if (it->hasConveyorPartExceededLimit(threshold))
        parts.push_back(*it);
    }
    return parts;
}

void availablePartsManager::addTypeCount(std::string type, int count=1){
  // whenever objects are added, add count. If vice versa, subtract count using negative values 
    std::map<std::string,int>::iterator it = type_count.find(type);
    if (it != type_count.end()){
      it->second += count;
    }
    else
      type_count.insert(std::pair<std::string,int>(type,1));
}

void availablePartsManager::addTypeIdPrev(std::string type, int id=1){
  // When a part is added, a new frame_id is assigned as per the ID of the map for the 
  // appropriate type. type count can be 2, but id can be >=2 depending on prev objects removed
    std::map<std::string,int>::iterator it = type_id_prev.find(type);
    if (it != type_id_prev.end()){
      it->second += id;
    }
    else
      type_id_prev.insert(std::pair<std::string,int>(type,0));
}

bool availablePartsManager::addPartFromLogicalCamera(nist_gear::Model m, std::string camera_frame){
  // First check if model type already exists. If not, add id and update count & prev id. 
  // Compute pose WRT world first.

    geometry_msgs::PoseStamped p, world_pose;
    p.header.frame_id = camera_frame;
    p.pose = m.pose;
    world_pose = tfBuffer.transform(p,"world");
    // std::cout << world_pose.pose.position.x << " " << world_pose.pose.position.y 
      // << " " << world_pose.pose.position.z << std::endl;
    if (getTypeCount(m.type) > 0){
      // Model type exists. Check position.
      std::vector<availablePart>::iterator it = availableParts.begin();
      while ( (it = std::find_if(it, availableParts.end(),
        [m](availablePart part) { return part.type==m.type; })) != availableParts.end() ){
          if (positionEquality(world_pose.pose,it->pose.pose)){
            // availableParts.push_back(availablePart(m.type,world_pose));
            // addTypeCount(m.type);
            return false; // Object exists, so we're not adding. 
          }
          it++;
      }
    }
    // If it exits the above block without returning, it means object is new and must be added
    std::string id = m.type + std::to_string(getTypeIdPrev(m.type) + 1);
    availableParts.push_back(availablePart(m.type, id, world_pose));
    // mostRecentPart = availablePart(m.type, id, world_pose);
    addTypeCount(m.type);
    addTypeIdPrev(m.type);
    /*ariac_2021_submission::addPlanningSceneCollision collision;
    collision.request.CollisionPose = world_pose.pose;
    collision.request.Dimensions = {0.12, 0.12, 0.05};  // Rough dimensions for now. Not sure if obj specific dimensions are necessary
    collision.request.ID = m.type + std::to_string(getTypeIdPrev(m.type) + 1);
    if(!addPlanningSceneCollisionKittingClient.call(collision)){
      ROS_ERROR_STREAM(id << " not added to planning collision scene!\n");
    }
    else
      std::cout << id << " added successfully by calling the service!" << std::endl;*/
      
    // std::cout << getTypeCount(m.type) << std::endl;
    return true;
    

    // geometry_msgs::TransformStamped p, world_pose;
    // p.child_frame_id = m.type + "_" + std::to_string(getTypeIdPrev(m.type)+1) + "_frame";
    // p.header.frame_id = camera_frame;
    // copyPoseToTransform(m.pose,p.transform);
    
    // std::vector<availablePart>::iterator it = availableParts.begin();
    // while ( (it = std::find_if(it, availableParts.end(),
    // [m](availablePart p) { return p.getType()==m.type; })) != availableParts.end() ){
      // if we find a product of same type in available parts matching the detected model

    //   if (!positionEquality(i->pose,(*it).pose))  // if not in same position
    //     it++;
    //   else{
    //     return true;
    //   }
    // }
    // return false;
}


/*class availableProducts{
  protected:
    std::vector<nist_gear::Product> products; // instead we can have a map of products and conveyor
    ros::Time lastPoseUpdateTime;
    
  public:
    void updatePoses();
    std::vector<nist_gear::Product> getProducts() { return products; }
    void addProductFromLogicalCam(nist_gear::Model);
    std::size_t numberOfAvailableProducts() { return products.size(); }

    // std::vector<std::string> getProductTypes(){
    //   std::vector<std::string> productTypes;
    //   for (auto it : products){
    //     productTypes.push_back(it.type);
    //   }
    //   return productTypes;
    // }
};


void availableProducts::addProductFromLogicalCam(nist_gear::Model m, std::string cam_id){
    nist_gear::Product p;
    std::string cam_frame = "logical_camera_" + cam_id + "_frame";

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = cam_frame;
    transformStamped.child_frame_id = m.type + std::to_string(2);
    try{
      transformStamped = tfBuffer.lookupTransform(frame, "world",
                              ros::Time(0));
      bin_transforms[i] = transformStamped;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      success = false;
    }

  //   logical_camera_1_2_frame
  // logical_camera_3_4_frame
    // Tf broadcaster service must be advertised by a class. 
    p.type = m.type;
    p.pose = m.pose;

    products.push_back(p);

}
*/


// --------------------------------- KITTING CLASS -----------------------------------

struct kittingProduct{
    nist_gear::Product product;
    bool required;
    bool partReorientation;
    kittingProduct(nist_gear::Product p, bool r, bool part_reorientation = false) : product(p), required(r), partReorientation(part_reorientation) { }
    geometry_msgs::Pose getProductDestinationPose(int kit_id /*, std::string agv_id*/);
};


geometry_msgs::Pose kittingProduct::getProductDestinationPose(int kit_id /*, std::string agv_id*/){
    geometry_msgs::PoseStamped t;
    t.header.frame_id = "kit_tray_" + std::to_string(kit_id);
    // std::cout << "Using kit_id : " << t.header.frame_id << ", agv id : " << agv_id << std::endl;
    t.pose = product.pose;
    return tfBuffer.transform(t, "world").pose;
}

class kittingTask{
  public:
    // kittingTask(const nist_gear::KittingShipment& ks) : shipment(ks) { }
    std::vector<kittingProduct> kitting_products;
    kittingTask() { }
    kittingTask(const nist_gear::KittingShipment& ks, bool newHighPriority = false) 
      : shipment_type(ks.shipment_type), agv_id(ks.agv_id), station_id(ks.station_id), newHighPriorityOrder(newHighPriority) {
        if (agv_id.size() > 0)
          kit_id = int(agv_id[agv_id.size()-1] - '0');
        for (auto i : ks.products){
          kitting_products.push_back(kittingProduct(i,true));
          // products.insert(std::pair<nist_gear::Product, bool>(i, true));
        }
    }
    // kit_tray_1, etc
    geometry_msgs::Pose getProductDestinationPose(std::string type); //Useless! // Always call after verifying availability
    geometry_msgs::Pose getProductDestinationPose();  // Useless
    int getReqTypeCount(std::string type);
    int getReqCount();
    std::vector<std::string> getRequiredProductTypes();
    std::vector<kittingProduct> getRequiredKittingProducts();
    bool isNewHighPriority() { return newHighPriorityOrder; }
    int getKitID() { return kit_id; }
    std::string getAgvID() { return agv_id; }
    bool isFinished() { return finished; }
    void setFinished(bool Fin) { finished = Fin; }
    int getPriority() { return kittingPriority; }
    void setPriority(int priority) { kittingPriority = priority; }
    void markProductReqByIndex(bool req, int index);
    // bool areJobsAllotted() { return jobsAllotted; }
    // void setJobsAllotted(bool b) { jobsAllotted = b; }
    void sendAGV();
    
  private:
    // nist_gear::KittingShipment shipment;
    std::string shipment_type, agv_id, station_id;
    int kit_id;
    bool finished = false;
    bool newHighPriorityOrder = false;
    // bool jobsAllotted = false;
    int kittingPriority = 1; // 1 is least important, higher is more important
    // std::vector<kittingProduct> kitting_products;
    // std::map<nist_gear::Product, bool> products; // product, and if it is required
};


std::vector<kittingProduct> kittingTask::getRequiredKittingProducts(){
    std::vector<kittingProduct> products;
    for (auto i : kitting_products)
      if (i.required)
        products.push_back(i);
    return products;
}

void kittingTask::sendAGV(){
  
}

void kittingTask::markProductReqByIndex(bool req, int index){
    kitting_products[index].required = req;
}

std::vector<std::string> kittingTask::getRequiredProductTypes(){
    std::vector<std::string> product_types;
    for (auto i : kitting_products){
      if (i.required)
        product_types.push_back(i.product.type);
    }
    return product_types;
}

int kittingTask::getReqCount(){
    int count = 0;
    for (auto i : kitting_products){
      if (i.required)
        count++;
    }
    return count;
}

geometry_msgs::Pose kittingTask::getProductDestinationPose(std::string product_type){
    std::vector<kittingProduct>::iterator it = kitting_products.begin();
    // geometry_msgs::Pose world_pose;
    // For multiple products of same type, use line below, and it++
    // while ( (std::find_if(it, products.end(), [product_type] (std::pair<nist_gear::Product, bool> p) { return p.first.type == product_type; })) != products.end()){
    if ( (it = std::find_if(kitting_products.begin(), kitting_products.end(), [product_type] (kittingProduct p) { return ( (p.product.type == product_type) && p.required); })) != kitting_products.end()){
      geometry_msgs::PoseStamped t;
      t.header.frame_id = "kit_tray_" + std::to_string(kit_id);
      std::cout << "Using kit_id : " << t.header.frame_id << ", agv id : " << agv_id << std::endl;
      t.pose = it->product.pose;
      return tfBuffer.transform(t, "world").pose;
    }
    else{
      ROS_ERROR_STREAM("Requested product type not required for kitting!");
      return geometry_msgs::Pose();
    }
}

geometry_msgs::Pose kittingTask::getProductDestinationPose(){
    if (kitting_products.size() > 0){
      geometry_msgs::PoseStamped t;
      t.header.frame_id = "kit_tray_" + std::to_string(kit_id);
      std::cout << "Using kit_id : " << t.header.frame_id << ", agv id : " << agv_id << std::endl;
      t.pose = kitting_products[0].product.pose;
      return tfBuffer.transform(t, "world").pose;
    }
    else
      ROS_ERROR_STREAM("Kitting required products are empty at the moment!");
}

int kittingTask::getReqTypeCount(std::string product_type){
    std::vector<kittingProduct>::iterator it = kitting_products.begin();
    int count = 0;
    while ( (std::find_if(it, kitting_products.end(), [product_type] (kittingProduct p) { return ( (p.product.type == product_type) && p.required); })) != kitting_products.end()){
      it++;
      count++;
    }
    return count;
}




// ----------------------------- ENUM CLASS ------------------------------------------

class enumClass{
  public:
    enum ROBOT{
      kitting, assembly, any
    };

    enum TASK{
      pick_and_place, assemble, part_reorientation
    };

    enum EVENT{
      NEW_LC_PART_DETECTED, NEW_UNKNOWN_PART_DETECTED, CONVEYOR_THRESHOLD_REACHED,
      FAULTY_SENSOR, ROBOT_HEALTH_FAILED, UPDATE_PENDING
    };
};








// --------------------------------- TASKMANAGER CLASS ---------------------------------------

class TaskManager{
  public:

    // ----------------------------- JOB CLASS ------------------------------------------

    class job{
      // A job object contains availablePart, assigned robot, etc. 
      // Used to perform kitting/assembly/part_reorientation by calling services.
      private:
        availablePart part;
        std::string type_ID;
        // std::string part_type;
        enumClass::ROBOT robot;
        enumClass::TASK task;
        geometry_msgs::Pose final_pose;
        bool success = false;
        boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>> pickupStaticClient;
        bool resultReceived = false;
        ros::NodeHandlePtr _nh_action;

      public:
        job() { }
        job(const job& j) : part(j.part), type_ID(j.type_ID), robot(j.robot), task(j.task), final_pose(j.final_pose), success(j.success), _nh_action(j._nh_action) {
          // _nh = boost::make_shared<ros::NodeHandle>();
          pickupStaticClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>>(*_nh_action, "ariac/kitting/pick_and_place_static_action");
          pickupStaticClient->waitForServer();
        }
        // job(std::string partType, geometry_msgs::Pose finalPose, enumClass::ROBOT r = enumClass::ROBOT::kitting, 
        //   enumClass::TASK t = enumClass::TASK::pick_and_place) : part_type(partType), robot(r), task(t), final_pose(finalPose) { }
        job(availablePart p, geometry_msgs::Pose finalPose, std::string ID, ros::NodeHandlePtr& nodeHandle,
          enumClass::ROBOT r = enumClass::ROBOT::kitting, 
          enumClass::TASK t = enumClass::TASK::pick_and_place) : part(p), robot(r), task(t), final_pose(finalPose), _nh_action(nodeHandle) {
            pickupStaticClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>>(*_nh_action, "ariac/kitting/pick_and_place_static_action");
            pickupStaticClient->waitForServer();
        }
        void perform();
        bool isPerforming();
        bool gotResult() { return resultReceived; }
        bool isSuccess() { return success; }
        std::string getTypeID() { return type_ID; }
        void setPickUpStaticClient(boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>> client) { pickupStaticClient = client; }
        enumClass::TASK getTask() { return task; }
        void setTask(enumClass::TASK T) { task = T; }
        enumClass::ROBOT getRobot() { return robot; }
        void setRobot(enumClass::ROBOT R) { robot = R; }
        void sppFeedbackCb(const ariac_2021_submission::pickupStaticObjectFeedbackConstPtr& feedback);
        void sppDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::pickupStaticObjectResultConstPtr& result);
        void sppActiveCb();
        void performSpinOnce();

        
    };

    // --------------------------------- JOB INTERFACE STRUCT --------------------------------------

    struct jobInterface{
      public:
        std::vector<job> currentJobs;
        std::vector<job> pendingJobs;
        std::vector<job> highPriorityJobs;
        // std::vector<job> currentlyImpossiblePendingJobs;
        bool assigned = false;
      
        jobInterface() { }
        // int getCurrentJobsCount() { return currentJobs.size(); }
        // int getpendingJobsCount() { return pendingJobs.size(); }
        // int gethighPriorityJobsCount() { return highPriorityJobs.size(); }
        // int getImpossibleJobsCount() { return currentlyImpossiblePendingJobs.size(); }
        int getJobCountByTask(enumClass::TASK Task);
        bool isCurrentRobotInAction(enumClass::ROBOT r);   // If 
        bool areCurrentRobotsInAction();
        bool currentJobGotResult();
        std::vector<enumClass::ROBOT> getRobotsNotInAction();
        int getPendingJobCountByRobot(enumClass::ROBOT R);
        job eraseAndGetPendingJobByRobot(enumClass::ROBOT R);
        void spinCurrentJobs();

    };


    TaskManager(ros::NodeHandlePtr& nh) : _nh(nh) {
        tfListenerPtr = boost::make_shared<tf2_ros::TransformListener>(tfBuffer);
        nh_action = boost::make_shared<ros::NodeHandle>();
        addPlanningSceneCollisionKittingClient = _nh->serviceClient<ariac_2021_submission::addPlanningSceneCollision>("ariac/kitting/add_kitting_planning_scene_collision");
        addPlanningSceneCollisionKittingClient.waitForExistence(ros::Duration(10));
        order_sub = _nh->subscribe("ariac/orders", 4, &TaskManager::orderCallback, this);
        lc_conveyor_sub = _nh->subscribe("/ariac/logical_camera_conveyor", 10, &TaskManager::lcConveyorCallback, this);
        // lc_1_2_sub = _nh->subscribe("/ariac/logical_camera_1_2", 3, &TaskManager::lc12Callback, this);
        // lc_3_4_sub = _nh->subscribe("/ariac/logical_camera_3_4", 3, &TaskManager::lc34Callback, this);
        lc_1_2 = _nh->createTimer(ros::Duration(2), &TaskManager::lc_1_2_timer_callback, this);
        lc_3_4 = _nh->createTimer(ros::Duration(2), &TaskManager::lc_3_4_timer_callback, this);
        while (!updateBinTransforms()){
          std::cout << "Unable to update bin transforms!" << std::endl;
        }
        start_competition(_nh);
        // pickupStaticObjectClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>>("ariac/kitting/pick_and_place_static_action");
        // pickupStaticObjectClient->waitForServer();
        while (availablePartsManagerInstance.getAvailableParts().size() == 0){
          // std::cout << "No parts available yet" << std::endl;
          ros::spinOnce();
        }
        ros::Rate rate(0.4);
        while (_nh->ok()){
          // summa();
          jobAllocator();
          ros::spinOnce(); // Need to test if fast subscriberCallbacks also get called without this.
          rate.sleep();
        }
    }


  private:
    ros::NodeHandlePtr _nh;
    ros::NodeHandlePtr nh_action;
    ros::Subscriber order_sub;
    ros::Subscriber lc_conveyor_sub;
    // ros::Subscriber lc_1_2_sub;
    // ros::Subscriber lc_3_4_sub;
    ros::Timer lc_1_2;
    ros::Timer lc_3_4;
    availablePartsManager availablePartsManagerInstance;
    // boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::pickupStaticObjectAction>> pickupStaticObjectClient;
    // tf2_ros::Buffer tfBuffer;
    // boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    std::vector<geometry_msgs::TransformStamped> bin_transforms;
    std::vector<kittingTask> kittingTasks;
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_1_2;
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_3_4;
    bool kittingPickAndPlaceInProgress = false;
    // std::vector<job> currentJobs;
    // std::vector<job> pendingJobs;
    // std::vector<job> highPriorityJobs;
    // std::vector<job> currentlyImpossiblePendingJobs;
    jobInterface orderJobInterface;
    void lc_1_2_timer_callback(const ros::TimerEvent& e);
    void lc_3_4_timer_callback(const ros::TimerEvent& e);
    void orderCallback(const nist_gear::OrderConstPtr& orderMsg);
    void lcConveyorCallback(const nist_gear::LogicalCameraImageConstPtr& msg);
    bool isProductAvailable(std::vector<nist_gear::Model>::const_iterator i, std::vector<nist_gear::Product> _currentAvailableParts);
    void printStaticTF();
    bool updateBinTransforms();
    bool kittingHighPriorityExists();
    bool updateKittingFinished();   // sets kT to finished and sends AGV if reqCount = 0. returns true if all kTs are finished.
    int getPartBin(availablePart p);
    void samplePickAndPlace();
    void jobAllocator();
    void eventManager(enumClass::EVENT e);
    // void summa();
    // void lc12Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
    // void lc34Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
};

void TaskManager::eventManager(enumClass::EVENT e){
    std::cout << "eventManager called!" << std::endl;
    switch(e){

      // for each kitting product in kittingTasks, if there is a non-required availablePart of same type,
      // we mark it as req_for_kitting and create a pending job with 
      /*case enumClass::EVENT::NEW_LC_PART_DETECTED :
        // std::cout << "Inside new LC detected case" << std::endl;
        for(auto it : kittingTasks){
          std::vector<kittingProduct> products = it.getRequiredKittingProducts();
          std::cout << products.size() << " kitting products are required" << std::endl;
          for (std::vector<kittingProduct>::iterator product = products.begin(); product != products.end(); product++){
            if (!product->partReorientation && product->required){
              // std::cout << "Part doesn't require reorientation" << std::endl;
              if (availablePartsManagerInstance.getNonRequiredTypeCount(product->product.type) > 0){
                std::cout << "Adding pending job!" << std::endl;
                availablePart part = availablePartsManagerInstance.getNonRequiredPart(product->product.type, true);
                orderJobInterface.pendingJobs.push_back( job(part, product->getProductDestinationPose(it.getKitID()), part.type_id, pickupStaticObjectClient,
                  enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place) );
                it.markProductReqByIndex(false, (product - products.begin()));    // Mark kittingProduct as non-required.
              }
              else
                std::cout << availablePartsManagerInstance.getNonRequiredTypeCount(product->product.type) << std::endl;
            }
          }
        }*/
        // std::cout << "Quit new LC detected case" << std::endl;

      case enumClass::EVENT::UPDATE_PENDING :
        for (auto& it : kittingTasks){
          for (std::size_t i = 0; i < it.kitting_products.size(); i++){
            if (it.kitting_products[i].required && availablePartsManagerInstance.getNonRequiredTypeCount(it.kitting_products[i].product.type) > 0){
              std::cout << "-------------------------------" << std::endl;
              std::cout << "Adding pending job!" << std::endl;
              availablePart part = availablePartsManagerInstance.getNonRequiredPart(it.kitting_products[i].product.type, true);
              orderJobInterface.pendingJobs.push_back( job(part, it.kitting_products[i].getProductDestinationPose(it.getKitID()), 
                part.type_id, _nh, enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place));
              std::cout << "BEFORE : " << it.getReqCount() << std::endl;
              it.kitting_products[i].required = false;
              std::cout << "AFTER : " << it.getReqCount() << std::endl;
            }
          }
        }
    }
}


void TaskManager::jobAllocator(){
    std::cout << "jobAllocator called" << std::endl;
    orderJobInterface.spinCurrentJobs();
    eventManager(enumClass::UPDATE_PENDING);
    std::cout << orderJobInterface.currentJobs.size() << " , " << orderJobInterface.pendingJobs.size() << std::endl;
    availablePartsManagerInstance.updateTransforms();
    if (orderJobInterface.currentJobs.size() > 0){
      if (orderJobInterface.currentJobGotResult()){   // if any of current jobs got result, if a pending job of same robot type exists, then we erase it and pushback a pending job
        for (std::vector<job>::iterator i = orderJobInterface.currentJobs.begin(); i!=orderJobInterface.currentJobs.end(); i++){
          if ( (*i).gotResult() && (*i).isSuccess() && orderJobInterface.getPendingJobCountByRobot((*i).getRobot()) > 0 ){
            std::cout << "Replacing current finished job!" << std::endl;
            job newCurrentJob = job( orderJobInterface.eraseAndGetPendingJobByRobot((i->getRobot())) );
            newCurrentJob.perform();
            availablePartsManagerInstance.setInAction(newCurrentJob.getTypeID(), true);
            orderJobInterface.currentJobs.erase(i);
            orderJobInterface.currentJobs.push_back( newCurrentJob );
          }
        }
      }
    }
    // if no currentJobs and no pendingJobs exist, we check if kittingTasks have been finished and marked as finished. If not marked as finished, we send AGV and mark as finished.
    else{
      if (orderJobInterface.pendingJobs.size() == 0){
        for (auto kT : kittingTasks){
          if (kT.getReqCount() == 0 && !kT.isFinished()){
            kT.sendAGV();
            kT.setFinished(true);
          }
        }
      }
    }

    // allocate pendingJob to robot not in action. Case of pendingJobs > 0 regardless of currentJobs size
    if (!orderJobInterface.areCurrentRobotsInAction()){
      std::cout << "Some robot not in action!" << std::endl;
      for (auto robot : orderJobInterface.getRobotsNotInAction()){
        if (orderJobInterface.getPendingJobCountByRobot(robot) > 0){
          std::cout << "Adding new current job!" << std::endl;
          job newCurrentJob = job( orderJobInterface.eraseAndGetPendingJobByRobot(robot) );
          newCurrentJob.perform();
          orderJobInterface.currentJobs.push_back( newCurrentJob );
        }
      }
    }

    // if (kittingTasks.size() > 0){
    //   if (orderJobInterface.currentJobs.size() == 0 && orderJobInterface.pendingJobs.size() == 0 && orderJobInterface.highPriorityJobs.size() == 0){
    //     // if (!kittingHighPriorityExists()){
    //     //   for (auto kt : kittingTasks){
    //     //     if (kt.getReqCount() == 0){
    //     //       kt.sendAGV();
    //     //       kt.setFinished(true);
    //     //     }
    //     //   }
    //     // }
    //   }
    // }
}

/*void TaskManager::samplePickAndPlace(){
    // pickupStaticObjectClient
    std::cout << "Sample pick and place fn called" << std::endl;
    ariac_2021_submission::pickupStaticObjectGoal goal;
    if (kittingTasks.size() > 0)
      availablePartsManagerInstance.updateKittableParts(kittingTasks[0].getRequiredProductTypes());
    availablePart a = availablePartsManagerInstance.getAvailableParts()[0];
    // while (availablePartsManagerInstance.getAvailableParts().size() == 0){
    //   std::cout << "No parts available yet" << std::endl;
    // }
    // std::cout << "New part found! Picking and placing it!" << std::endl;
    // if (availablePartsManagerInstance.getAvailableParts().size() > 0)
    //   a = availablePartsManagerInstance.getAvailableParts()[0];
    // else
    //   ROS_ERROR_STREAM("No available parts to pick and place!");
    goal.ID = a.type_id;
    goal.supporting_surface = "bin" + std::to_string(getPartBin(a)) + "_center_collision";
    goal.initial_pose = a.pose.pose;
    goal.final_pose = kittingTasks[0].getProductDestinationPose();
    if (kittingTasks.size() > 0){

    }
    pickupStaticObjectClient->sendGoal(goal);
    pickupStaticObjectClient->waitForResult();
    // while (!pickupStaticObjectClient->getResult()->success){
    //   continue;
    // }
    std::cout << "Server side returned success = " << pickupStaticObjectClient->getResult()->success << std::endl;
    if (pickupStaticObjectClient->getResult()->success)
      std::cout << "YAYYYYY" << std::endl;
}*/

bool TaskManager::kittingHighPriorityExists(){
    for (auto i : kittingTasks){
      if (i.isNewHighPriority())
        return true;
    }
    return false;
}

bool TaskManager::updateKittingFinished(){
    std::vector<kittingTask>::iterator it;
    // If there exists any kT that has no required parts but isn't marked as finished
    // if (it = (std::find_if(kittingTasks.begin(), kittingTasks.end(), [] (kittingTask k) { return (!k.isFinished() && k.getReqCount()==0); }) != kittingTasks.end() ))

}


bool TaskManager::updateBinTransforms(){
    geometry_msgs::TransformStamped transformStamped;
    bool success = true;
    bin_transforms.resize(8);
    for(int i=0; i<8; i++) {
      std::string frame = "bin" + std::to_string(i+1) + "_frame";
      try{
        transformStamped = tfBuffer.lookupTransform(frame, "world",
                                ros::Time(0));
        bin_transforms[i] = transformStamped;
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        success = false;
      }
    }
    return success;
}

int TaskManager::getPartBin(availablePart p){
    // if x-y translations of object wrt bin < 0.3. Get pose of model wrt bin
    
    if (updateBinTransforms()){
      for (std::size_t i=0; i<bin_transforms.size(); i++){
        if (isPositionDiffLesser(bin_transforms[i].transform.translation,p.pose.pose.position,0.3)) {
          // std::cout << m.type << " is in Bin : " << i+1 << std::endl;
          return i+1;
        }
        // else
        //   std::cout << m.type << "is not in Bin : " << i+1 << std::endl;
      }
      std::cout << "No bins found?" << std::endl;
      return 0;
    }
}

void TaskManager::printStaticTF() {
    geometry_msgs::TransformStamped transformStamped;
    std::cout << "Inside printStaticTF() " << std::endl;
    try{
      transformStamped = tfBuffer.lookupTransform("bin1_frame", "world",
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // std::cout << transformStamped.transform.translation.x << " " << transformStamped.transform.translation.y << " " << transformStamped.transform.translation.z << " " << std::endl;

}

// bool TaskManager::isProductAvailable(std::vector<nist_gear::Model>::const_iterator i, std::vector<nist_gear::Product> _currentAvailableParts){
//   std::vector<nist_gear::Product>::iterator it = _currentAvailableParts.begin();
//   while ( (it = std::find_if(it,_currentAvailableParts.end(),
//     [i](nist_gear::Product p) { return p.type==i->type; })) != _currentAvailableParts.end() ){
//       // if we find a product of same type in available parts matching the detected model
//       if (!positionEquality(i->pose,(*it).pose))  // if not in same position
//         it++;
//       else{
//         return true;
//       }
//   }
//   return false;
// }



void TaskManager::lc_1_2_timer_callback(const ros::TimerEvent& e) {
  if (kittingTasks.size() > 0){
    msg_1_2 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2");
    // std::cout << "---------------------------------------------" << std::endl;
    for (std::vector<nist_gear::Model>::const_iterator i = msg_1_2->models.begin(); i!=msg_1_2->models.end(); i++){
      // std::cout << availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_1_2_frame") << std::endl;
      availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_1_2_frame");
        // eventManager(enumClass::EVENT::NEW_LC_PART_DETECTED);
    // std::vector<nist_gear::Product> _currentAvailableParts = availableParts.getProducts();
    // for (std::vector<nist_gear::Model>::const_iterator i = msg_1_2->models.begin(); i!=msg_1_2->models.end(); i++){
    //   if (!isProductAvailable(i,_currentAvailableParts)) {
    //     availableParts.addProductFromLogicalCam(*i,);
    //     // getModelBin(*i);
    //   }
    }
  }
    // std::cout << availablePartsManagerInstance.getAvailableParts().size() << std::endl;
    // for (auto it : availablePartsManagerInstance.getAvailableParts())
    //   std::cout << getPartBin(it) << std::endl;
    // std::cout << "Number of available products 1_2 = " << availableParts.numberOfAvailableProducts() << std::endl;
    // printStaticTF();
    // std::cout << "Callback 1_2, Time : " << ros::Time::now().sec << std::endl;

}

void TaskManager::lc_3_4_timer_callback(const ros::TimerEvent& e) {
    if (kittingTasks.size() > 0){
      msg_3_4 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4");
      // std::vector<nist_gear::Product> _currentAvailableParts = availableParts.getProducts();
      // std::cout << "---------------------------------------------" << std::endl;
      for (std::vector<nist_gear::Model>::const_iterator i = msg_3_4->models.begin(); i!=msg_3_4->models.end(); i++){
        // std::cout << availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_3_4_frame") << std::endl;
        availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_3_4_frame");
      
      //   if (!isProductAvailable(i,_currentAvailableParts)) {
      //     availableParts.addProductFromLogicalCam(*i);
      //     getModelBin(*i);
      //   }
      }
    }
    // std::cout << availablePartsManagerInstance.getAvailableParts().size() << std::endl;
    // for (auto it : availablePartsManagerInstance.getAvailableParts())
    //   std::cout << getPartBin(it) << std::endl;
    // std::cout << "Number of available products 3_4 = " << availableParts.numberOfAvailableProducts() << std::endl;
    // std::cout << "Callback 3_4, Time : " << ros::Time::now().sec << std::endl;
}

void TaskManager::orderCallback(const nist_gear::OrderConstPtr& orderMsg){
    std::cout << "\033[1;34mNew Order Recieved!\033[0m" << std::endl;

    for (auto it=orderMsg->kitting_shipments.begin(); it!=orderMsg->kitting_shipments.end(); ++it){
      kittingTasks.push_back(kittingTask(*it));
    }
    // Need to add for assembly shipments
    std::cout << "Size of kittingTasks is " << kittingTasks.size() << std::endl;
    // std::cout << "Req count before assigning : " << kittingTasks[0].getReqCount() << std::endl;
    // kittingTasks[0].kitting_products[0].required = false;
    // std::cout << "Req count after assigning : " << kittingTasks[0].getReqCount() << std::endl;

}

void TaskManager::lcConveyorCallback(const nist_gear::LogicalCameraImageConstPtr& msg){
    // std::cout << "lcConveyor subscriber callback called" << std::endl;
}

// void TaskManager::lc12Callback(const nist_gear::LogicalCameraImageConstPtr& msg){

//     // std::cout << "lc12 subscriber callback called" << std::endl;
// }

// void TaskManager::lc34Callback(const nist_gear::LogicalCameraImageConstPtr& msg){
//     // std::cout << "lc34 subscriber callback called" << std::endl;
// }
void TaskManager::job::perform(){
    std::cout << "Perform called!" << std::endl;
    if (task == enumClass::TASK::pick_and_place){
      ariac_2021_submission::pickupStaticObjectGoal goal;
      goal.ID = type_ID;
      goal.initial_pose = part.pose.pose;
      goal.final_pose = final_pose;
      // std::cout << "Trying to connect to server!" << std::endl;
      // pickupStaticClient->waitForServer();
      pickupStaticClient->sendGoal(goal, boost::bind(&job::sppDoneCb, this, _1, _2), boost::bind(&job::sppActiveCb, this), boost::bind(&job::sppFeedbackCb, this, _1));
      // pickupStaticClient->waitForResult();
    }
}

void TaskManager::job::sppFeedbackCb(const ariac_2021_submission::pickupStaticObjectFeedbackConstPtr& feedback){
    std::cout << "FEEDBACK!" << std::endl;
}

void TaskManager::job::sppDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::pickupStaticObjectResultConstPtr& result){
    std::cout << "Pick and place for object " << type_ID << " finished with state : " << state.toString().c_str() << std::endl;
    if (result->success){
      std::cout << "SUCCESS!" << std::endl;
      resultReceived = true;
      success = true;
    }
    else{
      std::cout << "FAILED!" << std::endl;
      resultReceived = true;
      success = false;
    }
}

void TaskManager::job::sppActiveCb(){
    std::cout << "Goal has been sent!" << std::endl;
}

void TaskManager::job::performSpinOnce(){
    ros::spinOnce();
}

int TaskManager::jobInterface::getJobCountByTask(enumClass::TASK Task){

}

bool TaskManager::jobInterface::isCurrentRobotInAction(enumClass::ROBOT r){
    if (std::find_if(currentJobs.begin(), currentJobs.end(), [r] (job j) { return (j.getRobot() == r); }) != currentJobs.end())
      return true;
    return false;
}

bool TaskManager::jobInterface::areCurrentRobotsInAction(){
    if (isCurrentRobotInAction(enumClass::ROBOT::kitting) && isCurrentRobotInAction(enumClass::ROBOT::assembly))
      return true;
    return false;
}

bool TaskManager::jobInterface::currentJobGotResult(){
    for (auto i : currentJobs){
      if (i.gotResult())
        return true;
    }
    return false;
}

std::vector<enumClass::ROBOT> TaskManager::jobInterface::getRobotsNotInAction(){
    std::vector<enumClass::ROBOT> robots;
    if (!isCurrentRobotInAction(enumClass::ROBOT::kitting))
      robots.push_back(enumClass::ROBOT::kitting);
    if (!isCurrentRobotInAction(enumClass::ROBOT::assembly))
      robots.push_back(enumClass::ROBOT::assembly);
    return robots;
}

int TaskManager::jobInterface::getPendingJobCountByRobot(enumClass::ROBOT R){
    int count = 0;
    for (auto j : pendingJobs){
      if (j.getRobot() == R)
        count++;
    }
    return count;
}

TaskManager::job TaskManager::jobInterface::eraseAndGetPendingJobByRobot(enumClass::ROBOT R){
    job j;
    for (std::vector<job>::iterator i = pendingJobs.begin(); i!=pendingJobs.end(); i++)
      if (i->getRobot() == R){
        j = job(*i);
        pendingJobs.erase(i);
        return j;
      }
}

void TaskManager::jobInterface::spinCurrentJobs(){
    for (auto i : currentJobs){
      i.performSpinOnce();
    }
}

/* Important frames :
  bin1_frame - bin8_frame
  base_link (kitting)
  briefcase_1 - briefcase_4
  kit_tray_1 - kit_tray_4
  logical_camera_1_2_frame
  logical_camera_3_4_frame
  logical_camera_conveyor_frame */

  