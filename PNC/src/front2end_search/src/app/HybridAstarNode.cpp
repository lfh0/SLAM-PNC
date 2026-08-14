#include "path_searching/hybridastar.h"

#include <exception>

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hybridastar");
    ros::NodeHandle node_handle;

    try {
        path_searching::HybridAstar hybrid_astar;
        hybrid_astar.init(node_handle);
        ros::spin();
    } catch (const std::exception& exception) {
        ROS_FATAL("HybridAstar initialization failed: %s", exception.what());
        return 1;
    }
    return 0;
}
