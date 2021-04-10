#include <iostream>
#include <algorithm>
#include <vector>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/KittingShipment.h>
#include <nist_gear/Product.h>

#include <tf2_ros/transform_listener.h>
// #include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
// #include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf/transform_datatypes.h>

tf2_ros::Buffer tfBuffer;
boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;

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


struct availablePart{
  
    std::string type;
    std::string frame_id;
    bool moving;
    bool conveyor;
    geometry_msgs::PoseStamped pose;
    // geometry_msgs::TransformStamped estimated_pose;
    ros::Time lastPoseUpdateTime;
    double conveyorVel = 0.2;
    int bin_id = 0;

    availablePart() {
      lastPoseUpdateTime = ros::Time::now();
    }
    availablePart(std::string in_type,geometry_msgs::PoseStamped in_pose) 
      : type(in_type), pose(in_pose) {

        lastPoseUpdateTime = ros::Time::now();      
    }
    std::string getType() { return type; }
    bool isMoving() { return moving; }
    void setMoving(bool mv) { moving = mv; }
    bool onConveyor() { return conveyor; }
    void setConveyor(bool conv) { conveyor = conv; }
    void setBin(int bin) { bin_id = bin; }
    void estimateAndUpdateTransform();
    

};

void availablePart::estimateAndUpdateTransform(){
    if (conveyor){
      ros::Duration diff = ros::Time::now() - lastPoseUpdateTime;
      pose.pose.position.y -= conveyorVel*(diff.sec+diff.nsec*pow(10,-9));
    }
}

class availablePartsManager{
  private:
    std::vector<availablePart> availableParts;
    std::map<std::string,int> type_count;
    std::map<std::string,int> type_id_prev;
  public:
    availablePartsManager() {
      
    }
    void update();
    void updateTransforms();
    void addTypeCount(std::string type, int count);
    void addTypeIdPrev(std::string type, int id);
    int getTypeCount(std::string type);
    int getTypeIdPrev(std::string type);
    std::vector<availablePart> getAvailableParts() { return availableParts;}
    bool addPartFromLogicalCamera(nist_gear::Model m, std::string camera_frame);

};


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

void availablePartsManager::updateTransforms() {
    for (std::vector<availablePart>::iterator it = availableParts.begin(); it!=availableParts.end(); it++){
      it->estimateAndUpdateTransform();
    }
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
    std::map<std::string,int>::iterator it = type_count.find(type);
    if (it != type_count.end()){
      it->second += id;
    }
    else
      it->second = id;
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
        [m](availablePart part) { return part.getType()==m.type; })) != availableParts.end() ){
          if (positionEquality(world_pose.pose,it->pose.pose)){
            // availableParts.push_back(availablePart(m.type,world_pose));
            // addTypeCount(m.type);
            return false; // Object exists, so we're not adding. 
          }
          it++;
      }
    }
    // If it exits the above block without returning, it means object is new and must be added
    availableParts.push_back(availablePart(m.type,world_pose));
    addTypeCount(m.type);
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

class kittingTask{
  public:
    // kittingTask(const nist_gear::KittingShipment& ks) : shipment(ks) { }
    kittingTask(const nist_gear::KittingShipment& ks) 
      : shipment_type(ks.shipment_type), agv_id(ks.agv_id), station_id(ks.station_id),
        products(ks.products) {  }
  private:
    // nist_gear::KittingShipment shipment;
    std::string shipment_type, agv_id, station_id;
    std::vector<nist_gear::Product> products;
};

class TaskManager{
  public:
    TaskManager(ros::NodeHandlePtr& nh) : _nh(nh) {
        tfListenerPtr = boost::make_shared<tf2_ros::TransformListener>(tfBuffer);
        order_sub = _nh->subscribe("ariac/orders", 4, &TaskManager::orderCallback, this);
        lc_conveyor_sub = _nh->subscribe("/ariac/logical_camera_conveyor", 10, &TaskManager::lcConveyorCallback, this);
        // lc_1_2_sub = _nh->subscribe("/ariac/logical_camera_1_2", 3, &TaskManager::lc12Callback, this);
        // lc_3_4_sub = _nh->subscribe("/ariac/logical_camera_3_4", 3, &TaskManager::lc34Callback, this);
        lc_1_2 = _nh->createTimer(ros::Duration(5), &TaskManager::lc_1_2_timer_callback, this);
        lc_3_4 = _nh->createTimer(ros::Duration(5), &TaskManager::lc_3_4_timer_callback, this);
        while (!updateBinTransforms()){
          std::cout << "Unable to update bin transforms!" << std::endl;
        }
        start_competition(_nh);
    }
    

  private:
    ros::NodeHandlePtr _nh;
    ros::Subscriber order_sub;
    ros::Subscriber lc_conveyor_sub;
    // ros::Subscriber lc_1_2_sub;
    // ros::Subscriber lc_3_4_sub;
    ros::Timer lc_1_2;
    ros::Timer lc_3_4;
    void lc_1_2_timer_callback(const ros::TimerEvent& e);
    void lc_3_4_timer_callback(const ros::TimerEvent& e);
    std::vector<kittingTask> kittingTasks;
    void orderCallback(const nist_gear::OrderConstPtr& orderMsg);
    void lcConveyorCallback(const nist_gear::LogicalCameraImageConstPtr& msg);
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_1_2;
    boost::shared_ptr<const nist_gear::LogicalCameraImage> msg_3_4;
    bool isProductAvailable(std::vector<nist_gear::Model>::const_iterator i, std::vector<nist_gear::Product> _currentAvailableParts);
    availablePartsManager availablePartsManagerInstance;
    // tf2_ros::Buffer tfBuffer;
    // boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;
    std::vector<geometry_msgs::TransformStamped> bin_transforms;
    void printStaticTF();
    bool updateBinTransforms();
    int getPartBin(availablePart p);

    // void lc12Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
    // void lc34Callback(const nist_gear::LogicalCameraImageConstPtr& msg);
};

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
    msg_1_2 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_1_2");
    std::cout << "---------------------------------------------" << std::endl;
    for (std::vector<nist_gear::Model>::const_iterator i = msg_1_2->models.begin(); i!=msg_1_2->models.end(); i++){
      // std::cout << availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_1_2_frame") << std::endl;
      availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_1_2_frame");
    // std::vector<nist_gear::Product> _currentAvailableParts = availableParts.getProducts();
    // for (std::vector<nist_gear::Model>::const_iterator i = msg_1_2->models.begin(); i!=msg_1_2->models.end(); i++){
    //   if (!isProductAvailable(i,_currentAvailableParts)) {
    //     availableParts.addProductFromLogicalCam(*i,);
    //     // getModelBin(*i);
    //   }
    }
    // std::cout << availablePartsManagerInstance.getAvailableParts().size() << std::endl;
    // for (auto it : availablePartsManagerInstance.getAvailableParts())
    //   std::cout << getPartBin(it) << std::endl;
    // std::cout << "Number of available products 1_2 = " << availableParts.numberOfAvailableProducts() << std::endl;
    // printStaticTF();
    // std::cout << "Callback 1_2, Time : " << ros::Time::now().sec << std::endl;

}

void TaskManager::lc_3_4_timer_callback(const ros::TimerEvent& e) {
    msg_3_4 = ros::topic::waitForMessage<nist_gear::LogicalCameraImage>("/ariac/logical_camera_3_4");
    // std::vector<nist_gear::Product> _currentAvailableParts = availableParts.getProducts();
    std::cout << "---------------------------------------------" << std::endl;
    for (std::vector<nist_gear::Model>::const_iterator i = msg_3_4->models.begin(); i!=msg_3_4->models.end(); i++){
      // std::cout << availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_3_4_frame") << std::endl;
      availablePartsManagerInstance.addPartFromLogicalCamera((*i),"logical_camera_3_4_frame");
    //   if (!isProductAvailable(i,_currentAvailableParts)) {
    //     availableParts.addProductFromLogicalCam(*i);
    //     getModelBin(*i);
    //   }
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


/* Important frames :
  bin1_frame - bin8_frame
  base_link (kitting)
  briefcase_1 - briefcase_4
  kit_tray_1 - kit_tray_4
  logical_camera_1_2_frame
  logical_camera_3_4_frame
  logical_camera_conveyor_frame */

  