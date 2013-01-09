#include "os5000/os5000_core.h"

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "compass_node");
    ros::NodeHandle n;
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

    // Create a new OSCompass object.
    OS5000 *oscompass = new OS5000(portname, baud, rate, init_time);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<os5000::os5000Config> gain_srv;
    dynamic_reconfigure::Server<os5000::os5000Config>::CallbackType f;
    f = boost::bind(&OS5000::configCallback, oscompass, _1, _2);
    gain_srv.setCallback(f);

    // Set up publishers.
    oscompass->createPublisher(&pnh);

    // Tell ROS to run this node at the rate that the compass is sending messages to us.
    ros::Rate r(rate);

    // Connect to the Ocean Server compass.
    if (!oscompass->isConnected())
    {
        ROS_ERROR("Could not connect to compass on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", portname.c_str(), baud);
    }

    // Main loop.
    while (n.ok())
    {
        // Get compass data.
        if (oscompass->isConnected())
        {
            oscompass->getData();

            float current_yaw = oscompass->getYaw();
            if (current_yaw > 180.)
            {
                current_yaw -= 360.;
                oscompass->setYaw(current_yaw);
            }

            // Publish the message.
            oscompass->publishImuData();

        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
