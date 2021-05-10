#include <kittingRobot.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kittingRobot_node");
    ros::NodeHandlePtr node(new ros::NodeHandle);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    kittingRobot kittingRobotInstance(node);
    std::cout << "Exited kittingRobotInstance" << std::endl;
    
}