#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");

    ros::NodeHandle nh;

    std::string world_link;

// Default value version
    nh.param<std::string>("/world_link", world_link, "default_value");

    ROS_INFO_STREAM(world_link);

    return 0;
}