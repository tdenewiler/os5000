#include "os5000/os5000_ros.h"

OSCompass::OSCompass(std::string _portname, int _baud, int _rate, int _init_time) : Compass::Compass(_portname, _baud, _rate, _init_time)
{
}

OSCompass::~OSCompass()
{
}

void OSCompass::publishImuData(ros::Publisher *pub_imu_data)
{
    sensor_msgs::Imu imudata;
    double linear_acceleration_covariance = 10000.;
    double angular_velocity_covariance = 10000.;
    double orientation_covariance = 1.;

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    imudata.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll * M_PI / 180., pitch * M_PI / 180., yaw * M_PI / 180.);
    
    imudata.orientation_covariance[0] = orientation_covariance;
    imudata.orientation_covariance[4] = orientation_covariance;
    imudata.orientation_covariance[8] = orientation_covariance;

    imudata.angular_velocity.x = 0.;
    imudata.angular_velocity.y = 0.;
    imudata.angular_velocity.z = 0.;

    imudata.angular_velocity_covariance[0] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[4] = angular_velocity_covariance;
    imudata.angular_velocity_covariance[8] = angular_velocity_covariance;

    imudata.linear_acceleration.x = 0.;
    imudata.linear_acceleration.y = 0.;
    imudata.linear_acceleration.z = 0.;

    imudata.linear_acceleration_covariance[0] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[4] = linear_acceleration_covariance;
    imudata.linear_acceleration_covariance[8] = linear_acceleration_covariance;

    ROS_DEBUG("OS5000 Quaternions = %.1f, %.1f, %.1f, %.1f", imudata.orientation.x, imudata.orientation.y, imudata.orientation.z, imudata.orientation.w);
    ROS_DEBUG("OS5000 (RPY) = (%lf, %lf, %lf)", roll, pitch, yaw);

    imudata.header.stamp = ros::Time::now();

    pub_imu_data->publish(imudata);

    // Update transform for sensor state.
    transform.header.stamp = ros::Time::now();
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = imudata.orientation.x;
    transform.transform.rotation.y = imudata.orientation.y;
    transform.transform.rotation.z = imudata.orientation.z;
    transform.transform.rotation.w = imudata.orientation.w;

    // Send the transform describing current sensor state.
    tf_broadcaster.sendTransform(transform);
}

void OSCompass::configCallback(os5000::os5000Config &config, uint32_t level)
{
    ROS_INFO("Reconfiguring port, baud, rate, init_time, reconnect to %s, %d, %d, %d, %d", config.port.c_str(), config.baud, config.rate, config.init_time, config.reconnect);

    // Set class variables to new values.
    baud      = config.baud;
    init_time = config.init_time;
    portname  = config.port.c_str();
    rate      = config.rate;

    // Check to see if we should attempt to reconnect to the compass.
    if (config.reconnect)
    {
        // Use the new compass settings to reconnect.
        setup();
        ROS_INFO("Using new settings to reconnect to compass. Got fd = %d", fd);

        // Reset the reconnect variable.
        config.reconnect = false;
    }
}

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
    OSCompass *oscompass = new OSCompass(portname, baud, rate, init_time);

    // Set up a dynamic reconfigure server.
    dynamic_reconfigure::Server<os5000::os5000Config> gain_srv;
    dynamic_reconfigure::Server<os5000::os5000Config>::CallbackType f;
    f = boost::bind(&OSCompass::configCallback, oscompass, _1, _2);
    gain_srv.setCallback(f);

    // Set up publishers.
    ros::Publisher pubImuData = pnh.advertise<sensor_msgs::Imu>("data", 1);

    // Tell ROS to run this node at the rate that the compass is sending messages to us.
    ros::Rate r(rate);

    // Connect to the Ocean Server compass.
    if (oscompass->fd < 0)
    {
        ROS_ERROR("Could not connect to compass on port %s at %d baud. You can try changing the parameters using the dynamic reconfigure gui.", oscompass->portname.c_str(), oscompass->baud);
    }

    // Main loop.
    while (n.ok())
    {
        // Get compass data.
        if (oscompass->fd > 0)
        {
            oscompass->getData();

            if (oscompass->yaw > 180.)
            {
                oscompass->yaw -= 360.;
            }

            // Publish the message.
            oscompass->publishImuData(&pubImuData);

        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
