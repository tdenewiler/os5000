#include "os5000/os5000_core.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "os5000_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Local variables.
    int baud;
    int init_time;
    std::string portname;
    int rate;

    // Initialize node parameters.
    pnh.param("baud",      baud,      int(115200));
    pnh.param("init_time", init_time, int(3));
    pnh.param("port",      portname,  std::string("/dev/ttyUSB0"));
    pnh.param("rate",      rate,      int(40));
    if (rate <= 0)
    {
        rate = 1;
    }

    // Create a new OSCompass object.
    OS5000 *oscompass = new OS5000(portname, baud, rate, init_time);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<os5000::os5000Config> gain_srv;
    dynamic_reconfigure::Server<os5000::os5000Config>::CallbackType f;
    f = boost::bind(&OS5000::configCallback, oscompass, _1, _2);
    gain_srv.setCallback(f);

    // Set up publishers.
    oscompass->createPublisher(&pnh);

    // Create a timer callback.
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/rate), &OS5000::timerCallback, oscompass);

    ros::spin();

    return 0;
}
