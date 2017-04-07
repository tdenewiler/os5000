#include <os5000/os5000_core.h>

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "os5000");

  ros::NodeHandle nh;
  os5000::serial::OS5000 node(nh);

  ros::spin();

  return 0;
}
