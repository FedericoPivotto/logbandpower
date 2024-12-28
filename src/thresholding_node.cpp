/* Standard libraries */
#include <ros/ros.h>

/* User-defined libraries */
#include <logbandpower/thresholding.h>

int main(int argc, char **argv) {
    // Node "thresholding_node" with arguments
    ros::init(argc, argv, "thresholding_node");

    // Spin node
    ros::spin();

    return 0;
}