#include <ros/ros.h>

#include "driver.h"

int DEBUG_LEVEL = 3;

using namespace bipedlab;

int main(int argc, char *argv[])
{
    // // ros
    ros::init(argc, argv, "driver");
    ros::NodeHandle nh("~");

    Driver* driver = new Driver(nh);
    std::cout<< "ALL DONE" << std::endl;
    return 0;
}
