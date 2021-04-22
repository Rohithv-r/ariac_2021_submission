#include <ARIACTaskManager.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv) {
    
    ros::init(argc, argv, "task_manager_node");
    ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>(); // why is this wrong? Gotta figure out initialization
    // ros::NodeHandlePtr node(new ros::NodeHandle);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    TaskManager tm(node);
    std::cout << "Quit Taskmanager object!" << std::endl;
    ros::waitForShutdown();
    // ros::spin();
    // ros::Rate rate(10);
    // ros::Publisher pub = node->advertise<std_msgs::Bool>("summa_topic",2);
    // std_msgs::Bool b;
    // b.data = true;
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // geometry_msgs::TransformStamped transformStamped;
    // while (node->ok()){
        // pub.publish(b);
        // b.data = !(b.data);
        // try{
        //     transformStamped = tfBuffer.lookupTransform("logical_camera_1_2_frame", "world", ros::Time(0));
        //     // logical_camera_3_4_frame, logical_camera_conveyor_frame
        // }
        // catch (tf2::TransformException &ex) {
        //     ROS_WARN("%s",ex.what());
        //     ros::Duration(1.0).sleep();
        //     continue;
        // }
        // std::cout << transformStamped. << std::endl;

        // rate.sleep();
    // }
    // ros::waitForShutdown();

}