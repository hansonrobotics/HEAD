#include <manyears_ros/manyears_node.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manyears_node");

    ros::NodeHandle n, np("~");
    manyears_ros::ManyEarsNode node(n, np);

    ros::spin();

    return 0;
}

