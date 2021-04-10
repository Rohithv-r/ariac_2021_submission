#include <kitting_robot_sample.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "ariac_example_node");

    ros::NodeHandle node;

    // Instance of custom class from above.
    MyCompetitionClass comp_class(node);

    comp_class.send_arm_to_zero_state();
    ros::spin();
}