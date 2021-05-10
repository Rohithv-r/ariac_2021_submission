#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

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
#include <ariac_2021_submission/kittingPickUpStaticObjectAction.h>
#include <ariac_2021_submission/kittingPickUpConveyorObjectAction.h>
#include <ariac_2021_submission/inspectionAssemblyAction.h>


tf2_ros::Buffer tfBuffer;
boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
ros::ServiceClient addPlanningSceneCollisionKittingClient;
// ros::CallbackQueue action_queue;

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
    bool convCrossedThreshold = false;
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
    bool setConvCrossedThreshold(bool crossed) { convCrossedThreshold = crossed; }

};

void availablePart::estimateAndUpdateTransform(){
    ros::Duration diff = ros::Time::now() - lastPoseUpdateTime;
    pose.pose.position.y -= conveyorVel*diff.toSec();
    lastPoseUpdateTime = ros::Time::now();
}

class availablePartsManager{
  private:
    std::vector<availablePart> availableParts;
    std::vector<availablePart> conveyorParts;
    std::vector<availablePart> AGVParts;
    std::map<std::string,int> type_count;
    std::map<std::string,int> type_id_prev;
    void addTypeCount(std::string type, int count);
    void addTypeIdPrev(std::string type, int id);

  public:
    availablePartsManager() {
      
    }
    // void update();
    // availablePart mostRecentPart;
    int newConvCrossed = 0;
    void updateTransforms();
    void logInfo();
    std::vector<availablePart> getConveyorPartsExceedingLimit();
    int getTypeCount(std::string type);
    int getNonRequiredTypeCount(std::string type);
    int getNonRequiredExceedingConveyorTypeCount(std::string type);
    availablePart getNonRequiredPart(std::string type, bool markAsRequired);  // Always call getNonRequiredTypeCount first
    availablePart getNonRequiredExceedingConveyorPart(std::string type, bool markAsRequired);
    int getTypeIdPrev(std::string type);
    std::vector<availablePart> getAvailableParts() { return availableParts;}
    std::vector<availablePart>& getConveyorParts() { return conveyorParts; }
    std::vector<availablePart> getAGVParts() { return AGVParts; }
    void conveyorToAvailablePart(std::string id);
    void eraseConveyorPart(std::string id);
    bool addPartFromLogicalCamera(nist_gear::Model m, std::string camera_frame, bool conveyor = false);
    void updateKittableParts(std::vector<std::string>);
    void setInAction(std::string typeID, bool inAction = true);
    availablePart getStaticKittablePart();
    std::vector<availablePart> getStaticKittableParts();
};

void availablePartsManager::logInfo(){
  std::cout << "----------------------------------------------------------" << std::endl;
  std::cout << "Available parts info : " << std::endl;
    for (auto i : availableParts){
      std::cout << i.type_id << ", " << i.inAction << ", " << i.req_for_kitting << std::endl;
    }
  std::cout << "Conveyor parts info : " << std::endl;
    for (auto i : conveyorParts){
      std::cout << i.type_id << ", " << i.inAction << ", " << i.req_for_kitting << std::endl;
    }
} 

int availablePartsManager::getNonRequiredTypeCount(std::string type){
    int count = 0;
    for (auto i : availableParts){
      if (!i.req_for_kitting && i.type == type){
        count++;
      }
    }
    return count;
}

int availablePartsManager::getNonRequiredExceedingConveyorTypeCount(std::string type){
    int count = 0;
    std::vector<availablePart> parts = getConveyorPartsExceedingLimit();
    for (auto i : parts){
      if (!i.req_for_kitting && i.type == type /*&& i.pose.pose.position.y > -4.2*/){
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
    for (auto &i : availableParts){
      if (!i.req_for_kitting){
        if (i.type == type){
          i.req_for_kitting = true;
          return i;
        }
      }
    }
}

availablePart availablePartsManager::getNonRequiredExceedingConveyorPart(std::string type, bool markAsRequired){
    availablePart p;
    for (std::vector<availablePart>::iterator it = conveyorParts.begin(); it!=conveyorParts.end(); it++){
      if (it->convCrossedThreshold && !it->req_for_kitting && it->type== type){
        std::cout << "Yaaaaqqqqqq" << std::endl;
        return *it;
      }
      else
        std::cout << "Noooqqq" << std::endl;
    }
    // return parts;
    // for (auto &i : parts){
    //   if (!i.req_for_kitting){
    //     if (i.type == type){
    //       i.req_for_kitting = true;
    //       std::cout << "YAAAAQAAA" << std::endl;
    //       return i;
    //     }
    //   }
    // }
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
      if (it->pose.pose.position.y < 0 && !it->convCrossedThreshold){
        it->setConvCrossedThreshold(true);
        newConvCrossed++;
      }
    }
}



std::vector<availablePart> availablePartsManager::getConveyorPartsExceedingLimit(){
    std::vector<availablePart> parts;
    for (std::vector<availablePart>::iterator it = conveyorParts.begin(); it!=conveyorParts.end(); it++){
      if (it->convCrossedThreshold)
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
      type_id_prev.insert(std::pair<std::string,int>(type,1));
}

void availablePartsManager::conveyorToAvailablePart(std::string id){
    std::vector<availablePart>::iterator it;
    if ( (it = std::find_if(conveyorParts.begin(), conveyorParts.end(), [id](availablePart p) { return p.type_id==id; })) != conveyorParts.end()){
      availablePart part = *it;
      availableParts.push_back(part);
      conveyorParts.erase(it);
    }
}

void availablePartsManager::eraseConveyorPart(std::string id){
    std::vector<availablePart>::iterator it;
    if ( (it = std::find_if(conveyorParts.begin(), conveyorParts.end(), [id](availablePart p) { return p.type_id==id; })) != conveyorParts.end()){
      conveyorParts.erase(it);
    }
}

bool availablePartsManager::addPartFromLogicalCamera(nist_gear::Model m, std::string camera_frame, bool conveyor){
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
          if (positionEquality(world_pose.pose, it->pose.pose)){
            // availableParts.push_back(availablePart(m.type,world_pose));
            // addTypeCount(m.type);
            return false; // Object exists, so we're not adding. 
          }
          it++;
      }
    }
    // If it exits the above block without returning, it means object is new and must be added
    std::string id = m.type + std::to_string(getTypeIdPrev(m.type) + 1);
    if (!conveyor)
      availableParts.push_back(availablePart(m.type, id, world_pose));
    else
      conveyorParts.push_back(availablePart(m.type, id, world_pose));
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
      transformStamped = tfBuffer.lookupTransform("world", frame, 
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



// ---------------------------- ASSEMBLY CLASS ---------------------------------------

struct assemblyProduct{
    nist_gear::Product product;
    bool required;
    bool partReorientation;
    assemblyProduct(nist_gear::Product p, bool r, bool part_reorientation = false) : product(p), required(r), partReorientation(part_reorientation) { }
    // the below constructor is mainly useful when passing entire products as goal for dead reckoning.
    assemblyProduct(nist_gear::Product p, int bfcase_id, bool r, bool part_reorientation = false) : required(r), partReorientation(part_reorientation) {
      product = p;
      product.pose = getProductDestinationPose(bfcase_id);
    }
    geometry_msgs::Pose getProductDestinationPose(int briefcase_id /*, std::string agv_id*/);
};


geometry_msgs::Pose assemblyProduct::getProductDestinationPose(int briefcase_id /*, std::string agv_id*/){
    geometry_msgs::PoseStamped t;
    t.header.frame_id = "briefcase_" + std::to_string(briefcase_id);
    // std::cout << "Using kit_id : " << t.header.frame_id << ", agv id : " << agv_id << std::endl;
    t.pose = product.pose;
    return tfBuffer.transform(t, "world").pose;
}



class assemblyTask{
  public:
    // kittingTask(const nist_gear::KittingShipment& ks) : shipment(ks) { }
    std::vector<assemblyProduct> assembly_products;
    std::string shipment_type, station_id;
    assemblyTask() { }
    assemblyTask(const nist_gear::AssemblyShipment& as, bool newHighPriority = false) 
      : shipment_type(as.shipment_type), station_id(as.station_id) {
        if (station_id.size() > 0)
          briefcase_id = int(station_id[station_id.size()-1] - '0');
        for (auto i : as.products){
          assembly_products.push_back(assemblyProduct(i, briefcase_id, true));
          // products.insert(std::pair<nist_gear::Product, bool>(i, true));
        }
    }
    // kit_tray_1, etc
    int getReqTypeCount(std::string type);
    int getReqCount();
    int getBriefcaseID() { return briefcase_id; }
    std::vector<std::string> getRequiredProductTypes();
    std::vector<assemblyProduct> getRequiredAssemblyProducts();
    bool isFinished() { return finished; }
    void setFinished(bool Fin) { finished = Fin; }
    void markProductReqByIndex(bool req, int index);
    // bool areJobsAllotted() { return jobsAllotted; }
    // void setJobsAllotted(bool b) { jobsAllotted = b; }
    // void sendAGV();
    
  private:
    // nist_gear::KittingShipment shipment;
    // std::string shipment_type, station_id;
    int briefcase_id;
    bool finished = false;
    // bool jobsAllotted = false;
    int assemblyPriority = 1; // 1 is least important, higher is more important
    // std::vector<kittingProduct> kitting_products;
    // std::map<nist_gear::Product, bool> products; // product, and if it is required
};


std::vector<assemblyProduct> assemblyTask::getRequiredAssemblyProducts(){
    std::vector<assemblyProduct> products;
    for (auto i : assembly_products)
      if (i.required)
        products.push_back(i);
    return products;
}

void assemblyTask::markProductReqByIndex(bool req, int index){
    assembly_products[index].required = req;
}

std::vector<std::string> assemblyTask::getRequiredProductTypes(){
    std::vector<std::string> product_types;
    for (auto i : assembly_products){
      if (i.required)
        product_types.push_back(i.product.type);
    }
    return product_types;
}

int assemblyTask::getReqCount(){
    int count = 0;
    for (auto i : assembly_products){
      if (i.required)
        count++;
    }
    return count;
}


int assemblyTask::getReqTypeCount(std::string product_type){
    std::vector<assemblyProduct>::iterator it = assembly_products.begin();
    int count = 0;
    while ( (std::find_if(it, assembly_products.end(), [product_type] (assemblyProduct p) { return ( (p.product.type == product_type) && p.required); })) != assembly_products.end()){
      it++;
      count++;
    }
    return count;
}




// ----------------------------- ENUM CLASS ------------------------------------------

class enumClass{
  public:
    enum ROBOT{
      kitting, gantry, any
    };

    enum TASK{
      pick_and_place, conveyor_pick_and_place_to_empty, assemble, part_reorientation
    };

    enum EVENT{
      NEW_LC_PART_DETECTED, GRIPPER_FAILED, CONVEYOR_THRESHOLD_REACHED,
      FAULTY_SENSOR, ROBOT_HEALTH_FAILED, UPDATE_PENDING
    };

    enum CAMERA_ACTION{
      ADD_IF_NEW, FAULTY_GRIPPER_OBJ_MOVED
    };
};








// --------------------------------- TASKMANAGER CLASS ---------------------------------------

class TaskManager{
  public:

    // ----------------------------- JOB CLASS ------------------------------------------

    struct job{
      // A job object contains availablePart, assigned robot, etc. 
      // Used to perform kitting/assembly/part_reorientation by calling services.
      public:
        availablePart part;
        // std::string type_ID;
        // std::string part_type;
        enumClass::ROBOT robot;
        enumClass::TASK task;
        geometry_msgs::Pose final_pose;
        bool pending = true;
        bool success = false;
        bool inAction = false;
        // boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>> pickupStaticClient;
        bool resultReceived = false;
        // ros::NodeHandlePtr job_nh;
        // ros::Subscriber result_sub, feedback_sub;
        // TaskManager& Parent;

      
        // job(TaskManager& Tmg) : Parent(Tmg) { }
        job() { }
        job(availablePart& p, geometry_msgs::Pose finalPose, enumClass::ROBOT r = enumClass::ROBOT::kitting, 
          enumClass::TASK t = enumClass::TASK::pick_and_place) : part(p), robot(r), task(t), final_pose(finalPose) {
            // pickupStaticClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>(*job_nh, "ariac/kitting/pick_and_place_static_action");
            // pickupStaticClient->waitForServer();
        }
        /*job(const job& j) : part(j.part), type_ID(j.type_ID), robot(j.robot), task(j.task), final_pose(j.final_pose), success(j.success), job_nh(j.job_nh), pickupStaticClient(j.pickupStaticClient) {
          // _nh = boost::make_shared<ros::NodeHandle>();
          // pickupStaticClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>(*job_nh, "ariac/kitting/pick_and_place_static_action");
          // pickupStaticClient->waitForServer();
        }*/
        // job(std::string partType, geometry_msgs::Pose finalPose, enumClass::ROBOT r = enumClass::ROBOT::kitting, 
        //   enumClass::TASK t = enumClass::TASK::pick_and_place) : part_type(partType), robot(r), task(t), final_pose(finalPose) { }
        /*job(availablePart p, geometry_msgs::Pose finalPose, std::string ID, ros::NodeHandlePtr& nodeHandle, boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>& ppClient,
          enumClass::ROBOT r = enumClass::ROBOT::kitting, 
          enumClass::TASK t = enumClass::TASK::pick_and_place) : part(p), robot(r), task(t), final_pose(finalPose), job_nh(nodeHandle), pickupStaticClient(ppClient) {
            // pickupStaticClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>(*job_nh, "ariac/kitting/pick_and_place_static_action");
            // pickupStaticClient->waitForServer();
        }*/
        
        void perform();
        /*void setResultReceived(bool r) { resultReceived = r; }
        bool isPerformCalled() { return performCalled; };
        bool gotResult() { return resultReceived; }
        bool isSuccess() { return success; }
        std::string getPartType() { return part.type; }
        availablePart getPart() { return part; }
        geometry_msgs::Pose getFinalPose() { return final_pose; }*/
        // ros::NodeHandlePtr getNodeHandle() { return job_nh; }
        // boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>> getClient() { return pickupStaticClient; }
        // std::string getTypeID() { return type_ID; }
        // void setPickUpStaticClient(boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>> client) { pickupStaticClient = client; }
        /*enumClass::TASK getTask() { return task; }
        void setTask(enumClass::TASK T) { task = T; }
        enumClass::ROBOT getRobot() { return robot; }
        void setRobot(enumClass::ROBOT R) { robot = R; }*/
        // void sppFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback);
        // void sppResultCb(const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result);
        // void sppStatusCb(const actionlib::SimpleClientGoalState& state);
        /*void performSpinOnce();
        void sppActDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result);
        void sppActActiveCb();
        void sppActFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback);*/

        
    };

    // --------------------------------- JOB INTERFACE STRUCT --------------------------------------

    struct jobInterface{
      public:
        // std::vector<job> currentJobs;
        job currentKittingJob;
        bool currentKittingJobAssigned = false;
        job currentGantryJob;
        bool currentGantryJobAssigned = false;
        std::vector<job> pendingJobs;
        std::vector<job> convJobs;
        std::vector<job> highPriorityJobs;
        // std::vector<job> currentlyImpossiblePendingJobs;
      
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
        int getConvJobCount();
        job eraseAndGetPendingJobByRobot(enumClass::ROBOT R);
        job eraseAndGetConvJob();
        void eraseAndAddPendingJobByRobot(enumClass::ROBOT R);
        // std::vector<std::string> performCurrentJobs();
        void spinCurrentJobs();
        void logJobDetails();
        bool kittingRobotJobInProgress();
        bool gantryRobotJobInProgress();

    };

    TaskManager() { }
    TaskManager(ros::NodeHandlePtr& nh) : _nh(nh) {
        tfListenerPtr = boost::make_shared<tf2_ros::TransformListener>(tfBuffer);
        // nh_action = boost::make_shared<ros::NodeHandle>();
        addPlanningSceneCollisionKittingClient = _nh->serviceClient<ariac_2021_submission::addPlanningSceneCollision>("ariac/kitting/add_kitting_planning_scene_collision");
        // addPlanningSceneCollisionKittingClient.waitForExistence(ros::Duration(10));
        order_sub = _nh->subscribe("ariac/orders", 4, &TaskManager::orderCallback, this);
        lc_conveyor_sub = _nh->subscribe("/ariac/logical_camera_conveyor", 10, &TaskManager::lcConveyorCallback, this);
        // lc_1_2_sub = _nh->subscribe("/ariac/logical_camera_1_2", 3, &TaskManager::lc12Callback, this);
        // lc_3_4_sub = _nh->subscribe("/ariac/logical_camera_3_4", 3, &TaskManager::lc34Callback, this);
        // lc_1_2 = _nh->createTimer(ros::Duration(2), &TaskManager::lc_1_2_timer_callback, this);
        // lc_3_4 = _nh->createTimer(ros::Duration(2), &TaskManager::lc_3_4_timer_callback, this);
        while (!updateBinTransforms()){
          std::cout << "Unable to update bin transforms!" << std::endl;
        }
        kitting_robot_x_min = bin_transforms[0].transform.translation.x - 0.3;
        std::cout << "Kitting min x is : " << kitting_robot_x_min << std::endl;
        start_competition(_nh);
        kittingPickUpStaticObjectClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>("ariac/kitting/kitting_pick_and_place_static_action");
        // kittingPickUpStaticObjectClient->waitForServer();
        kittingPickUpConveyorObjectClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpConveyorObjectAction>>("ariac/kitting/kitting_pick_and_place_conveyor_action");
        // kittingPickUpConveyorObjectClient->waitForServer();
        gantryPickUpStaticObjectClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>>("ariac/gantry/gantry_pick_and_place_static_action");
        // gantryPickUpStaticObjectClient->waitForServer();
        inspectionAssemblyClient = boost::make_shared<actionlib::SimpleActionClient<ariac_2021_submission::inspectionAssemblyAction>>("ariac/gantry/inspection_assembly_action");
        /*while (availablePartsManagerInstance.getAvailableParts().size() == 0){
          // std::cout << "No parts available yet" << std::endl;
          ros::spinOnce();
        }*/

        /*while (_nh->ok()){
          sampleConveyorPickAndPlace();
        }*/

        /*sampleGantryPickAndPlace();
        while (_nh->ok()){
          ros::spinOnce();
        }*/

        // std::cout << "After sampleConvP&P!" << std::endl;
        ros::Rate rate(0.5);
        while (_nh->ok()){
          // summa();
          jobAllocator();
          // ros::spinOnce(); // Need to test if fast subscriberCallbacks also get called without this.
          rate.sleep();
        }
    }


  private:
    ros::NodeHandlePtr _nh;
    // ros::NodeHandlePtr nh_action;
    ros::Subscriber order_sub;
    ros::Subscriber lc_conveyor_sub;
    // ros::Subscriber lc_1_2_sub;
    // ros::Subscriber lc_3_4_sub;
    ros::Timer lc_1_2;
    ros::Timer lc_3_4;
    availablePartsManager availablePartsManagerInstance;
    boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>> kittingPickUpStaticObjectClient;
    boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpConveyorObjectAction>> kittingPickUpConveyorObjectClient;
    boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::kittingPickUpStaticObjectAction>> gantryPickUpStaticObjectClient;
    boost::shared_ptr<actionlib::SimpleActionClient<ariac_2021_submission::inspectionAssemblyAction>> inspectionAssemblyClient;
    // tf2_ros::Buffer tfBuffer;
    // boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    std::vector<geometry_msgs::TransformStamped> bin_transforms;
    std::vector<kittingTask> kittingTasks;
    std::vector<assemblyTask> assemblyTasks;
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_1_2;
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_3_4;
    bool kittingRobotInAction = false;
    bool gantryRobotInAction = false;
    bool conveyorPartDetected = false;
    double usualFinalZ = 0.751650;
    double kitting_robot_x_min = -1;
    std::map<std::string, double> objectHeights { {"pump", 0.126}, {"battery", 0.0533}, {"regulator", 0.0674}, {"sensor", 0.0674} };
    // std::vector<job> currentJobs;
    // std::vector<job> pendingJobs;
    // std::vector<job> highPriorityJobs;
    // std::vector<job> currentlyImpossiblePendingJobs;
    jobInterface orderJobInterface;
    void lc_1_2_timer_callback(const ros::TimerEvent& e);
    void lc_3_4_timer_callback(const ros::TimerEvent& e);
    int lc_new_count = 0;
    void lc_1_2_update(enumClass::CAMERA_ACTION action);
    void lc_3_4_update(enumClass::CAMERA_ACTION action);
    void orderCallback(const nist_gear::OrderConstPtr& orderMsg);
    void lcConveyorCallback(const nist_gear::LogicalCameraImageConstPtr& msg);
    bool isProductAvailable(std::vector<nist_gear::Model>::const_iterator i, std::vector<nist_gear::Product> _currentAvailableParts);
    void printStaticTF();
    bool updateBinTransforms();
    bool kittingHighPriorityExists();
    bool updateKittingFinished();   // sets kT to finished and sends AGV if reqCount = 0. returns true if all kTs are finished.
    int getPartBin(availablePart p);
    geometry_msgs::Pose getEmptyBinLocation(enumClass::ROBOT robot);
    void samplePickAndPlace();
    void sampleGantryPickAndPlace();
    void sampleConveyorPickAndPlace();
    void sampleInspectionAssembly();
    void jobAllocator();
    void eventManager(enumClass::EVENT e);
    void perform(job&);
    void kittingSPPDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result);
    void kittingSPPActiveCb();
    void kittingSPPFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback);
    void kittingConvDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpConveyorObjectResultConstPtr& result);
    void kittingConvActiveCb();
    void kittingConvFeedbackCb(const ariac_2021_submission::kittingPickUpConveyorObjectFeedbackConstPtr& feedback);
    void gantrySPPDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result);
    void gantrySPPActiveCb();
    void gantrySPPFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback);
    // void summa();
    // void lc12Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
    // void lc34Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
};

void TaskManager::eventManager(enumClass::EVENT e){
    // std::cout << "eventManager called!" << std::endl;
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
                orderJobInterface.pendingJobs.push_back( job(part, product->getProductDestinationPose(it.getKitID()), part.type_id, kittingPickUpStaticObjectClient,
                  enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place) );
                it.markProductReqByIndex(false, (product - products.begin()));    // Mark kittingProduct as non-required.
              }
              else
                std::cout << availablePartsManagerInstance.getNonRequiredTypeCount(product->product.type) << std::endl;
            }
          }
        }*/
        // std::cout << "Quit new LC detected case" << std::endl;

      case enumClass::EVENT::UPDATE_PENDING :       // Iterate through every kittingTask. For each kitting product, if it is required and there is a non-required type available, add a job for it in pendingJobs, mark both the kitting product as well as part as required.
        {
          for (auto& it : kittingTasks){
            for (std::size_t i = 0; i < it.kitting_products.size(); i++){
              if (it.kitting_products[i].required){
                if (availablePartsManagerInstance.getNonRequiredExceedingConveyorTypeCount(it.kitting_products[i].product.type) > 0){
                  std::cout << "*****************************************" << std::endl;
                  std::cout << "Adding conveyor pending job!" << std::endl;
                  std::cout << "Assigning a conveyor pending job with kitting robot" << std::endl;
                  availablePart part = availablePartsManagerInstance.getNonRequiredExceedingConveyorPart(it.kitting_products[i].product.type, true);
                  job j(part, it.kitting_products[i].getProductDestinationPose(it.getKitID()), 
                    enumClass::ROBOT::kitting, enumClass::TASK::conveyor_pick_and_place_to_empty);
                  orderJobInterface.convJobs.push_back( j );
                  it.kitting_products[i].required = false;
                }

                if (availablePartsManagerInstance.getNonRequiredTypeCount(it.kitting_products[i].product.type) > 0){
                  std::cout << "*****************************************" << std::endl;
                  std::cout << "Adding pending job!" << std::endl;
                  availablePart part = availablePartsManagerInstance.getNonRequiredPart(it.kitting_products[i].product.type, true);
                  // If part's x < kitting_x_min, then assign gantry to place it in an empty accessible bin?
                  if (part.pose.pose.position.x > kitting_robot_x_min){
                    std::cout << "Assigning a pending job with kitting robot" << std::endl;
                    job j(part, it.kitting_products[i].getProductDestinationPose(it.getKitID()), 
                      enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place);
                    orderJobInterface.pendingJobs.push_back( j );
                    it.kitting_products[i].required = false;
                  }
                  else{
                    std::cout << "Assigning a pending job with gantry robot" << std::endl;
                    job j(part, it.kitting_products[i].getProductDestinationPose(it.getKitID()), 
                      enumClass::ROBOT::gantry, enumClass::TASK::pick_and_place);
                    orderJobInterface.pendingJobs.push_back( j );
                    it.kitting_products[i].required = false;
                  }
                  std::cout << "Pending jobs size : " << orderJobInterface.pendingJobs.size() << std::endl;
                  usualFinalZ = it.kitting_products[i].getProductDestinationPose(it.getKitID()).position.z;
                  // std::cout << "BEFORE : " << it.getReqCount() << std::endl;
                  // it.kitting_products[i].required = false;
                  // std::cout << "AFTER : " << it.getReqCount() << std::endl;
                  // std::cout << orderJobInterface.pendingJobs[0].getTypeID() << " XXXXX" << std::endl;
                  std::cout << "*****************************************" << std::endl;
                }
              }
              
            }
          }
        break;
        }
        
      case enumClass::EVENT::CONVEYOR_THRESHOLD_REACHED : // conveyorJob becomes more important
        {
          std::vector<availablePart> convParts = availablePartsManagerInstance.getConveyorPartsExceedingLimit();
          for (auto& part : convParts){
            // orderJobInterface.convJobs.push_back( job(part, getEmptyBinLocation(enumClass::ROBOT::kitting), enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place));
            
          }
          break;
        }
        
      case enumClass::EVENT::GRIPPER_FAILED :
        {
          std::cout << "Gripper failed scenario passed into eventManager" << std::endl;
          lc_1_2_update(enumClass::CAMERA_ACTION::FAULTY_GRIPPER_OBJ_MOVED);
          lc_3_4_update(enumClass::CAMERA_ACTION::FAULTY_GRIPPER_OBJ_MOVED);
          break;
        }

    }
}


void TaskManager::jobAllocator(){
    // std::cout << "jobAllocator called" << std::endl;
    // orderJobInterface.spinCurrentJobs();
    if (lc_new_count < 3 || (availablePartsManagerInstance.getAvailableParts().size() == 0)){
      lc_1_2_update(enumClass::CAMERA_ACTION::ADD_IF_NEW);
      lc_3_4_update(enumClass::CAMERA_ACTION::ADD_IF_NEW);
    }
    eventManager(enumClass::UPDATE_PENDING);
    // std::cout << orderJobInterface.currentJobs.size() << " , " << orderJobInterface.pendingJobs.size() << std::endl;
    // std::cout << "Eeeebaaaa " << std::endl;
    availablePartsManagerInstance.updateTransforms();
    availablePartsManagerInstance.logInfo();
    if (!kittingRobotInAction){
      // std::cout << "Naaabaaaa" << std::endl;
      // std::cout << "YEY 1" << std::endl;
      // If there is high priority order, assign that. Else if there is a pendingJob for this robot, assign it. 
      if (orderJobInterface.getConvJobCount() > 0){
        orderJobInterface.currentKittingJob = orderJobInterface.eraseAndGetConvJob();
        std::cout << "Erased a conveyor job and assigned it to currentkittingJob. Size of pending = " << orderJobInterface.pendingJobs.size() << std::endl;
        orderJobInterface.currentKittingJobAssigned = true;
        perform(orderJobInterface.currentKittingJob);
      }      
      else if (orderJobInterface.getPendingJobCountByRobot(enumClass::ROBOT::kitting) > 0){
        orderJobInterface.currentKittingJob = orderJobInterface.eraseAndGetPendingJobByRobot(enumClass::ROBOT::kitting);
        std::cout << "Erased a pending job and assigned it to currentkittingJob. Size of pending = " << orderJobInterface.pendingJobs.size() << std::endl;
        orderJobInterface.currentKittingJobAssigned = true;
        perform(orderJobInterface.currentKittingJob);
      }
    }
    // else
    //   orderJobInterface.logJobDetails();

    if (!gantryRobotInAction){
      if (orderJobInterface.getPendingJobCountByRobot(enumClass::ROBOT::gantry) > 0){
        orderJobInterface.currentGantryJob = orderJobInterface.eraseAndGetPendingJobByRobot(enumClass::ROBOT::gantry);
        std::cout << "Erased a pending job and assigned it to currentGantryJob. Size of pending = " << orderJobInterface.pendingJobs.size() << std::endl;
        orderJobInterface.currentGantryJobAssigned = true;
        perform(orderJobInterface.currentGantryJob);
      }
    }


    /*// if no currentJob and no pendingJobs exist, we check if kittingTasks have been finished and marked as finished. If not marked as finished, we send AGV and mark as finished.
    else{
      if (orderJobInterface.pendingJobs.size() == 0){
        for (auto kT : kittingTasks){
          if (kT.getReqCount() == 0 && !kT.isFinished()){
            kT.sendAGV();
            kT.setFinished(true);
          }
        }
      }
      else{

      }
    }

    // allocate pendingJob to robot not in action. Case of pendingJobs > 0 regardless of currentJobs size
    if (!orderJobInterface.areCurrentRobotsInAction()){
      std::cout << "Some robot not in action!" << std::endl;
      for (auto robot : orderJobInterface.getRobotsNotInAction()){
        if (orderJobInterface.getPendingJobCountByRobot(robot) > 0){
          std::cout << "Adding new current job!" << std::endl;
          // orderJobInterface.eraseAndAddPendingJobByRobot(robot);
          for (std::vector<job>::iterator i = orderJobInterface.pendingJobs.begin(); i!=orderJobInterface.pendingJobs.end(); i++){
            if (i->getRobot() == robot){
              job j(i->getPart(), i->getFinalPose(), 
                i->getTypeID(), _nh, kittingPickUpStaticObjectClient, enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place);
              orderJobInterface.currentJobs.push_back(j);
              // std::cout << "EEEF " << orderJobInterface.pendingJobs.erase(i)->getPart().type << std::endl;
              std::cout << orderJobInterface.currentJobs[0].getPartType() << " @ " << orderJobInterface.pendingJobs[0].getPartType() << std::endl;
              break;
            }
          }
          std::vector<std::string> ids = orderJobInterface.performCurrentJobs();
          for (auto ID : ids)
            availablePartsManagerInstance.setInAction(ID, true);
          // newCurrentJob.perform();
          // orderJobInterface.currentJobs.push_back( newCurrentJob );
        }
      }
    }*/

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

// void TaskManager::jobAllocator(){
//     std::cout << "jobAllocator called" << std::endl;
//     // orderJobInterface.spinCurrentJobs();
//     eventManager(enumClass::UPDATE_PENDING);

// }

/*void TaskManager::samplePickAndPlace(){
    // kittingPickUpStaticObjectClient
    std::cout << "Sample pick and place fn called" << std::endl;
    ariac_2021_submission::kittingPickUpStaticObjectGoal goal;
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
    kittingPickUpStaticObjectClient->sendGoal(goal);
    kittingPickUpStaticObjectClient->waitForResult();
    // while (!kittingPickUpStaticObjectClient->getResult()->success){
    //   continue;
    // }
    std::cout << "Server side returned success = " << kittingPickUpStaticObjectClient->getResult()->success << std::endl;
    if (kittingPickUpStaticObjectClient->getResult()->success)
      std::cout << "YAYYYYY" << std::endl;
}*/

void TaskManager::sampleGantryPickAndPlace(){
    std::cout << "Sample gantry pick and place fn called" << std::endl;
    ariac_2021_submission::kittingPickUpStaticObjectGoal goal;
    while (availablePartsManagerInstance.getAvailableParts().size() == 0){
      // std::cout << "Waiting for conveyor parts!" << std::endl;
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
    gantryPickUpStaticObjectClient->waitForServer();
    availablePart a = availablePartsManagerInstance.getAvailableParts()[0];
    goal.ID = a.type_id;
    goal.initial_pose = a.pose.pose;
    goal.final_pose = getEmptyBinLocation(enumClass::ROBOT::kitting);
    
    gantryPickUpStaticObjectClient->sendGoal(goal);
    gantryPickUpStaticObjectClient->waitForResult();
    // while (!kittingPickUpStaticObjectClient->getResult()->success){
    //   continue;
    // }
    std::cout << "Server side returned success = " << gantryPickUpStaticObjectClient->getResult()->success << std::endl;
    if (gantryPickUpStaticObjectClient->getResult()->success)
      std::cout << "YAYYYYY" << std::endl;
}

void TaskManager::sampleConveyorPickAndPlace(){
    std::cout << "Sample conveyor pick and place fn called" << std::endl;
    ariac_2021_submission::kittingPickUpConveyorObjectGoal goal;
    // if (kittingTasks.size() > 0)
    //   availablePartsManagerInstance.updateKittableParts(kittingTasks[0].getRequiredProductTypes());
    // availablePart a = availablePartsManagerInstance.getAvailableParts()[0];
    // while (availablePartsManagerInstance.getAvailableParts().size() == 0){
    //   std::cout << "No parts available yet" << std::endl;
    // }
    // std::cout << "New part found! Picking and placing it!" << std::endl;
    // if (availablePartsManagerInstance.getAvailableParts().size() > 0)
    //   a = availablePartsManagerInstance.getAvailableParts()[0];
    // else
    //   ROS_ERROR_STREAM("No available parts to pick and place!");
    while (availablePartsManagerInstance.getConveyorParts().size() == 0){
      // std::cout << "Waiting for conveyor parts!" << std::endl;
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
    availablePartsManagerInstance.logInfo();
    availablePart part = availablePartsManagerInstance.getConveyorParts()[0];
    availablePartsManagerInstance.updateTransforms();
    goal.ID = part.type_id;
    for (auto it : objectHeights){
      if (part.type.find(it.first) != std::string::npos){
        goal.objectHeight = it.second;
        break;
      }
    }
    goal.initial_pose = part.pose.pose;
    goal.lastPoseUpdateTime = part.lastPoseUpdateTime;
    goal.final_pose = getEmptyBinLocation(enumClass::ROBOT::kitting);  // or kitting blah blah.
    
    kittingPickUpConveyorObjectClient->sendGoal(goal);
    std::cout << "GOAL SENT! WAITING FOR RESULT!" << std::endl;
    kittingPickUpConveyorObjectClient->waitForResult();
    // while (!kittingPickUpStaticObjectClient->getResult()->success){
    //   continue;
    // }
    bool success = kittingPickUpConveyorObjectClient->getResult()->success;
    // part.pose.pose = goal.final_pose;    // INSTEAD, it must be better to trigger the appropriate logical cameras and get the exact finalPose
    // availablePartsManagerInstance.conveyorToAvailablePart(part.type_id);
    availablePartsManagerInstance.eraseConveyorPart(part.type_id);
    std::cout << "Server side returned success = " << success << std::endl;
    if (success)
      std::cout << "YAYYYYY" << std::endl;
}

void TaskManager::sampleInspectionAssembly(){
    std::cout << "sampleInspectionAssembly called!" << std::endl;
    // inspectionAssemblyClient
    if (kittingTasks.size()==0 && assemblyTasks.size()>0){  // only inspection assem is possible
      ariac_2021_submission::inspectionAssemblyGoal goal;
      for (auto it : assemblyTasks){
        goal.station_id = it.station_id;
        std::vector<assemblyProduct> prod = it.getRequiredAssemblyProducts();
        for (auto j : prod)
          goal.req_products.push_back(j.product);
        inspectionAssemblyClient->sendGoal(goal);
        inspectionAssemblyClient->waitForResult();
      }
    }
}

void TaskManager::kittingSPPActiveCb(){
    std::cout << "Goal has been sent!" << std::endl;
}

void TaskManager::kittingSPPFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback){
    if (feedback->feedback == feedback->GRIPPER_FAULT)
      std::cout << "Gripper failed while performing current spp!" << std::endl;
}

void TaskManager::kittingSPPDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result){
    std::cout << "kitting SPP finished with state : " << state.toString().c_str() << std::endl;
    if (result->success){
      std::cout << "YESSSS. SUCCESSSSSS!" << std::endl;
      kittingRobotInAction = false;
    }
}

void TaskManager::gantrySPPActiveCb(){
    std::cout << "Goal has been sent!" << std::endl;
}

void TaskManager::gantrySPPFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback){
    if (feedback->feedback == feedback->GRIPPER_FAULT)
      std::cout << "Gripper failed while performing current spp!" << std::endl;
}

void TaskManager::gantrySPPDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result){
    std::cout << "gantry SPP finished with state : " << state.toString().c_str() << std::endl;
    if (result->success){
      std::cout << "YESSSS. SUCCESSSSSS!" << std::endl;
      gantryRobotInAction = false;
    }
}

void TaskManager::kittingConvActiveCb(){
    std::cout << "Goal has been sent!" << std::endl;
}

void TaskManager::kittingConvFeedbackCb(const ariac_2021_submission::kittingPickUpConveyorObjectFeedbackConstPtr& feedback){
    if (feedback->feedback == feedback->GRIPPER_FAULT)
      std::cout << "Gripper failed while performing current spp!" << std::endl;
}

void TaskManager::kittingConvDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpConveyorObjectResultConstPtr& result){
    std::cout << "kitting Conv finished with state : " << state.toString().c_str() << std::endl;
    if (result->success){
      std::cout << "YESSSS. SUCCESSSSSS!" << std::endl;
      kittingRobotInAction = false;
    }
    availablePartsManagerInstance.eraseConveyorPart(orderJobInterface.currentKittingJob.part.type_id);
}

void TaskManager::perform(job& currentJob){
    std::cout << "Perform called!" << std::endl;
    std::cout << "Passed job's robot is : " << currentJob.robot << std::endl;
    availablePartsManagerInstance.setInAction(currentJob.part.type_id, true);
    // currentJob.inAction = true;
    // pendingJob.part.inAction = true;
    if (currentJob.task == enumClass::TASK::pick_and_place){
      if (currentJob.robot == enumClass::ROBOT::kitting){
        kittingRobotInAction = true;
        ariac_2021_submission::kittingPickUpStaticObjectGoal goal;
        goal.ID = currentJob.part.type_id;
        goal.initial_pose = currentJob.part.pose.pose;
        goal.final_pose = currentJob.final_pose;
        std::cout << "Performing kitting p&p. Part type is : " << currentJob.part.type << std::endl;
        // std::cout << "Trying to connect to server!" << std::endl;
        kittingPickUpStaticObjectClient->waitForServer();
        kittingPickUpStaticObjectClient->sendGoal(goal, boost::bind(&TaskManager::kittingSPPDoneCb, this, _1, _2), boost::bind(&TaskManager::kittingSPPActiveCb, this), boost::bind(&TaskManager::kittingSPPFeedbackCb, this, _1));
        // feedback_sub = job_nh->subscribe("ariac/kitting/pick_and_place_static_action/feedback", 3, &job::sppFeedbackCb, this);
        // result_sub = job_nh->subscribe("ariac/kitting/pick_and_place_static_action/result", 3, &job::sppResultCb, this);
        // pickupStaticClient->waitForResult();
      }
      if (currentJob.robot == enumClass::ROBOT::gantry){
        gantryRobotInAction = true;
        ariac_2021_submission::kittingPickUpStaticObjectGoal goal;
        goal.ID = currentJob.part.type_id;
        goal.initial_pose = currentJob.part.pose.pose;
        goal.final_pose = currentJob.final_pose;
        std::cout << "Performing gantry p&p. Part type is : " << currentJob.part.type << std::endl;
        gantryPickUpStaticObjectClient->waitForServer();
        gantryPickUpStaticObjectClient->sendGoal(goal, boost::bind(&TaskManager::gantrySPPDoneCb, this, _1, _2), boost::bind(&TaskManager::gantrySPPActiveCb, this), boost::bind(&TaskManager::gantrySPPFeedbackCb, this, _1));
      }
    }

    else if (currentJob.task == enumClass::TASK::conveyor_pick_and_place_to_empty){
      if (currentJob.robot == enumClass::ROBOT::kitting){
        kittingRobotInAction = true;
        ariac_2021_submission::kittingPickUpConveyorObjectGoal goal;
        
        availablePartsManagerInstance.updateTransforms();
        goal.ID = currentJob.part.type_id;
        for (auto it : objectHeights){
          if (currentJob.part.type.find(it.first) != std::string::npos){
            goal.objectHeight = it.second;
            break;
          }
        }
        goal.initial_pose = currentJob.part.pose.pose;
        goal.lastPoseUpdateTime = currentJob.part.lastPoseUpdateTime;
        goal.final_pose = getEmptyBinLocation(enumClass::ROBOT::kitting);  // or kitting blah blah.
        // std::vector<availablePart> paaaarts = availablePartsManagerInstance.getConveyorParts();
        // std::vector<availablePart>::iterator p = std::find_if(paaaarts.begin(), paaaarts.end(), [goal] (availablePart k) { return goal.ID == k.type_id; })
        
        // std::cout << "DIFF BTW poses : " << goal.initial_pose.position.y-
        kittingPickUpConveyorObjectClient->sendGoal(goal, boost::bind(&TaskManager::kittingConvDoneCb, this, _1, _2), boost::bind(&TaskManager::kittingConvActiveCb, this), boost::bind(&TaskManager::kittingConvFeedbackCb, this, _1));
        std::cout << "CONVEYOR GOAL SENT!" << std::endl;
        // kittingPickUpConveyorObjectClient->waitForResult();
        // while (!kittingPickUpStaticObjectClient->getResult()->success){
        //   continue;
        // }
        // bool success = kittingPickUpConveyorObjectClient->getResult()->success;
        // part.pose.pose = goal.final_pose;    // INSTEAD, it must be better to trigger the appropriate logical cameras and get the exact finalPose
        // availablePartsManagerInstance.conveyorToAvailablePart(part.type_id);
        
        // std::cout << "Server side returned success = " << success << std::endl;
        // if (success)
        //   std::cout << "YAYYYYY" << std::endl;
      }
    }
}

/*void TaskManager::job::sppActActiveCb(){
    std::cout << "Goal has been sent!" << std::endl;
    std::cout << "Current part is actually " << part.type << " required? " << part.req_for_kitting << std::endl;
}

void TaskManager::job::sppActFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback){
    std::cout << "FEEDBACK RECEIVEDDDDDDDD!" << std::endl;
    if (feedback->feedback == feedback->PROBE_AND_PICK){
      std::cout << "Summa setting to finished and result true : " << std::endl;
      resultReceived = true;
      std::cout << "Current part is actually " << part.type << " required? " << part.req_for_kitting << std::endl;
    }
}

void TaskManager::job::sppActDoneCb(const actionlib::SimpleClientGoalState& state, const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result){
    std::cout << "FJT finished with state : " << state.toString().c_str() << std::endl;
    setResultReceived(true);
    std::cout << "EEHEE : " << gotResult() << std::endl;
    std::cout << "Part type is : " << part.type << std::endl;
    type_ID = "baloo";
    if (result->success){
      std::cout << "YESSSS. SUCCESSSSSS!" << std::endl;
      success = true;
    }
    else
      success = false;
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
        transformStamped = tfBuffer.lookupTransform("world", frame,
                                ros::Time(0));
        std::cout << "Bin transform : { " << transformStamped.transform.translation.x << ", " << transformStamped.transform.translation.y << ", " << transformStamped.transform.translation.z << " }" << std::endl;
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
    // std::cout << "LIL " << p.pose.pose.position.x << ", " << p.pose.pose.position.y << ", " << p.pose.pose.position.z << std::endl;
    // if (updateBinTransforms()){
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
    // }
}

struct two_dim_pt{
  double x, y;
  two_dim_pt(double X, double Y) : x(X), y(Y) { }
};

geometry_msgs::Pose TaskManager::getEmptyBinLocation(enumClass::ROBOT robot){
  std::cout << "getEmptyBinLocation called!" << std::endl;
  int bin;
  std::vector< std::vector<two_dim_pt> > bin_occupancy;
  double x_min, x_max, y_min, y_max, x, y;
  bool free_space;
  double l = 0.12;
  bin_occupancy.resize(bin_transforms.size());
  if (robot == enumClass::ROBOT::kitting){
    int nos[4] = {1, 2, 5, 6};
    for (auto it : availablePartsManagerInstance.getAvailableParts()){
      bin = getPartBin(it);
      if (bin == 1 || bin == 2 || bin == 5 || bin == 6){
        std::cout << "Occupancy pushback happening in bin : " << bin << std::endl;
        bin_occupancy[bin-1].push_back(two_dim_pt(it.pose.pose.position.x, it.pose.pose.position.y));
      }
    }
    for (int i = 3; i >= 0; i--){
    // for (int i=2; i>1; i--){
      x_min = bin_transforms[nos[i]-1].transform.translation.x - 0.3 + l;
      y_min = bin_transforms[nos[i]-1].transform.translation.y - 0.3 + l;
      x_max = bin_transforms[nos[i]-1].transform.translation.x + 0.3 - 0.09;
      y_max = bin_transforms[nos[i]-1].transform.translation.y + 0.3 - 0.09;
      for (x = x_min; x <= x_max; x = x+l){
        for (y = y_min; y <= y_max; y = y+l){
          if (bin_occupancy[nos[i]-1].size() > 0){    // TO DO : switch on/off logical camera while performing conveyor place, part reshuffling, etc.
            for (auto pt : bin_occupancy[nos[i]-1]){
              if ( (abs(x-pt.x) >= l) || (abs(y-pt.y) >= l) )
                free_space = true;
              else{
                free_space = false;
                std::cout << "DAH! x_diff, y_diff = " << abs(x-pt.x) << ", "  << abs(y-pt.y) << std::endl;
                break;
              }
            }
          }
          else{
            free_space = true;
            x = x_min;
            y = y_min;
          }
          if (free_space){
            std::cout << "Yaaah!  x, y, bin = " << x << ", " << y << ", " << nos[i] << std::endl;
            geometry_msgs::Pose p;
            p.position.x = x; p.position.y = y; p.position.z = usualFinalZ;
            p.orientation = quatFromRPY(0, 0, 0);
            return p;
          }
        }
        std::cout << "Next point y!" << std::endl;
      }
      std::cout << "Next point x!" << std::endl;
    }
    ROS_ERROR_STREAM("Could not find empty bin location!");
  }

}

void TaskManager::printStaticTF() {
    geometry_msgs::TransformStamped transformStamped;
    std::cout << "Inside printStaticTF() " << std::endl;
    try{
      transformStamped = tfBuffer.lookupTransform("world", "bin1_frame",
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

void TaskManager::lc_1_2_update(enumClass::CAMERA_ACTION action){
    // if (kittingTasks.size() > 0){
    if (action == enumClass::ADD_IF_NEW){
      msg_1_2 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2");
      for (std::vector<nist_gear::Model>::const_iterator i = msg_1_2->models.begin(); i!=msg_1_2->models.end(); i++){
        availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_1_2_frame");
      }
      lc_new_count++;
    }
}

void TaskManager::lc_3_4_update(enumClass::CAMERA_ACTION action){
    // if (kittingTasks.size() > 0){
    if (action == enumClass::ADD_IF_NEW){
      msg_3_4 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4");
      for (std::vector<nist_gear::Model>::const_iterator i = msg_3_4->models.begin(); i!=msg_3_4->models.end(); i++){
        availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_3_4_frame");
      }
    }
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
    for (auto it=orderMsg->assembly_shipments.begin(); it!=orderMsg->assembly_shipments.end(); ++it){
      assemblyTasks.push_back(assemblyTask(*it));
    }
    // Need to add for assembly shipments
    std::cout << "Size of kittingTasks is " << kittingTasks.size() << std::endl;
    // std::cout << "Req count before assigning : " << kittingTasks[0].getReqCount() << std::endl;
    // kittingTasks[0].kitting_products[0].required = false;
    // std::cout << "Req count after assigning : " << kittingTasks[0].getReqCount() << std::endl;

}

void TaskManager::lcConveyorCallback(const nist_gear::LogicalCameraImageConstPtr& msg){   // we assume that there are no more than 1 part in fov per msg
    // std::cout << "Conveyor parts size : " << availablePartsManagerInstance.getConveyorParts().size() << std::endl;
    if (msg->models.size() > 0 && !conveyorPartDetected){
      conveyorPartDetected = true;
      for (std::vector<nist_gear::Model>::const_iterator i = msg->models.begin(); i!=msg->models.end(); i++){
        availablePartsManagerInstance.addPartFromLogicalCamera((*i), "logical_camera_conveyor_frame", true);
      }
    }
    else{
      if (msg->models.size() == 0)
        conveyorPartDetected = false;
    }
    // std::cout << "lcConveyor subscriber callback called" << std::endl;
}

// void TaskManager::lc12Callback(const nist_gear::LogicalCameraImageConstPtr& msg){

//     // std::cout << "lc12 subscriber callback called" << std::endl;
// }

// void TaskManager::lc34Callback(const nist_gear::LogicalCameraImageConstPtr& msg){
//     // std::cout << "lc34 subscriber callback called" << std::endl;
// }
/*void TaskManager::job::perform(){
    std::cout << "Perform called!" << std::endl;
    performCalled = true;
    if (task == enumClass::TASK::pick_and_place){
      ariac_2021_submission::kittingPickUpStaticObjectGoal goal;
      goal.ID = part.type_id;
      goal.initial_pose = part.pose.pose;
      goal.final_pose = final_pose;
      std::cout << "Part type is : " << part.type << std::endl;
      // std::cout << "Trying to connect to server!" << std::endl;
      // pickupStaticClient->waitForServer();
      pickupStaticClient->sendGoal(goal, boost::bind(&job::sppActDoneCb, this, _1, _2), boost::bind(&job::sppActActiveCb, this), boost::bind(&job::sppActFeedbackCb, this, _1));
      // feedback_sub = job_nh->subscribe("ariac/kitting/pick_and_place_static_action/feedback", 3, &job::sppFeedbackCb, this);
      // result_sub = job_nh->subscribe("ariac/kitting/pick_and_place_static_action/result", 3, &job::sppResultCb, this);
      // pickupStaticClient->waitForResult();
    }
}*/

/*void TaskManager::job::sppFeedbackCb(const ariac_2021_submission::kittingPickUpStaticObjectFeedbackConstPtr& feedback){
    std::cout << "FEEDBACK!" << std::endl;
}

void TaskManager::job::sppResultCb(const ariac_2021_submission::kittingPickUpStaticObjectResultConstPtr& result){
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

void TaskManager::job::sppStatusCb(const actionlib::SimpleClientGoalState& state){
    // std::cout << "Goal has been sent!" << std::endl;
}*/

/*void TaskManager::job::performSpinOnce(){
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
}*/

/*bool TaskManager::jobInterface::currentJobGotResult(){
    for (auto i : currentJobs){
      if (i.gotResult()){
        std::cout << "Current got result!" << std::endl;
        return true;
      }
      else
        std::cout << "MEHHH" << std::endl;
    }
    return false;
}

std::vector<std::string> TaskManager::jobInterface::performCurrentJobs(){    // returns id of the availablePart being performed
    std::vector<std::string> ids;
    for (auto it : currentJobs){
      if (!it.isPerformCalled()){
        it.perform();
        ids.push_back(it.getTypeID());
      }
    }
    return ids;
}*/

void TaskManager::jobInterface::logJobDetails(){
    std::cout << "----------------------------------------------" << std::endl;
    // std::cout << "currentJobs details : " << std::endl;
    // for (auto& i : currentJobs){
    //   std::cout << i.getPartType() << ", " << i.getRobot() << " " << i.gotResult() << std::endl;
    //   // i.setResultReceived(true);
    // }
    std::cout << "pendingJobs details : " << std::endl;
    for (auto it : pendingJobs){
      std::cout << it.part.type_id << ", " << it.robot << ", " << it.resultReceived << std::endl;
    }
    std::cout << "currentKittingJob details : ";
    if (currentKittingJobAssigned)
      std::cout << currentKittingJob.part.type_id << ", " << currentKittingJob.robot << ", " << currentKittingJob.resultReceived << std::endl;
    else
      std::cout << std::endl;
    // }
}

bool TaskManager::jobInterface::currentJobGotResult(){
    if (currentKittingJob.resultReceived)
      return true;
    return false;
}

/*std::vector<enumClass::ROBOT> TaskManager::jobInterface::getRobotsNotInAction(){
    std::vector<enumClass::ROBOT> robots;
    if (!isCurrentRobotInAction(enumClass::ROBOT::kitting))
      robots.push_back(enumClass::ROBOT::kitting);
    if (!isCurrentRobotInAction(enumClass::ROBOT::assembly))
      robots.push_back(enumClass::ROBOT::assembly);
    return robots;
}*/

int TaskManager::jobInterface::getPendingJobCountByRobot(enumClass::ROBOT R){
    int count = 0;
    for (auto j : pendingJobs){
      if (j.robot == R)
        count++;
    }
    return count;
}

int TaskManager::jobInterface::getConvJobCount(){
    int count = 0;
    for (auto j : convJobs){
      if (j.part.pose.pose.position.y > -4)
        count++;
    }
    std::cout << "Kaaal : " << count << std::endl;
    return count;
}


TaskManager::job TaskManager::jobInterface::eraseAndGetPendingJobByRobot(enumClass::ROBOT R){
    job j;
    for (std::vector<job>::iterator i = pendingJobs.begin(); i!=pendingJobs.end(); i++)
      if (i->robot == R){
        j = job(i->part, i->final_pose, R);
        pendingJobs.erase(i);
        return j;
      }
}

TaskManager::job TaskManager::jobInterface::eraseAndGetConvJob(){
    job j;
    for (std::vector<job>::iterator i = convJobs.begin(); i!=convJobs.end(); i++)
      if (i->part.pose.pose.position.y > -4){
        j = job(i->part, i->final_pose, enumClass::ROBOT::kitting, enumClass::conveyor_pick_and_place_to_empty);
        convJobs.erase(i);
        return j;
      }
}

/*void TaskManager::jobInterface::eraseAndAddPendingJobByRobot(enumClass::ROBOT R){
    for (std::vector<job>::iterator i = pendingJobs.begin(); i!=pendingJobs.end(); i++)
      if (i->getRobot() == R){
        // job j(i->getPart(), i->getFinalPose(), 
        //         i->getTypeID(), i->getNodeHandle(), i->getClient(), enumClass::ROBOT::kitting, enumClass::TASK::pick_and_place);
        // currentJobs.push_back(j);
        pendingJobs.erase(i);
        return;
        // return j;
      }
}

void TaskManager::jobInterface::spinCurrentJobs(){
    for (auto i : currentJobs){
      i.performSpinOnce();
    }
}*/

/* Important frames :
  bin1_frame - bin8_frame
  base_link (kitting)
  briefcase_1 - briefcase_4
  kit_tray_1 - kit_tray_4
  logical_camera_1_2_frame
  logical_camera_3_4_frame
  logical_camera_conveyor_frame 
  quality_control_sensor_1_frame - 4
  */


// TO DO : switch on/off logical camera while performing conveyor place, part reshuffling, etc.

  