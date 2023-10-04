#include <drone.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "px4_drone_node");
    ros::NodeHandle nodehandle;

    ROS_INFO("px4 drone node start");
    Drone drone(nodehandle);
    drone._Run();

    return 0;
}