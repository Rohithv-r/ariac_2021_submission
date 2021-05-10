#include <gantryRobot.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "gantryRobot_node");
    ros::NodeHandlePtr node(new ros::NodeHandle);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    gantryRobot gantryRobotInstance(node);
    std::cout << "Exited gantryRobotInstance" << std::endl;
    
}