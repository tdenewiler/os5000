#include "os5000/os5000_core.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "os5000_node");
    ros::NodeHandle nh;

    OS5000 oscompass(nh);

    return 0;
}
