/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <logbandpower/bandpower.h>

int main(int argc, char **argv) {
    // Node "bandpower_node" with arguments
    ros::init(argc, argv, "bandpower_node");

    // Spin node
    ros::spin();

    return 0;
}