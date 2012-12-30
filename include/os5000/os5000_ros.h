#ifndef OS5000_ROS_H
#define OS5000_ROS_H

// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Local includes.
#include "os5000/os5000_core.h"
#include "os5000/timing.h"

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <os5000/os5000Config.h>

class OSCompass : public Compass
{
public:
    //! Constructor.
    OSCompass(std::string _portname, int _baud, int _rate, int _init_time);

    //! Destructor.
    ~OSCompass();

    //! Callback function for dynamic reconfigure server.
    void configCallback(os5000::os5000Config &config, uint32_t level);

    //! Publish the data from the compass in a ROS standard format.
    void publishImuData(ros::Publisher *pubImuData);

private:
    tf::TransformBroadcaster tf_broadcaster;
};

#endif // OS5000_ROS_H
