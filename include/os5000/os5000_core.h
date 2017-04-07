#ifndef OS5000_OS5000_CORE_H
#define OS5000_OS5000_CORE_H

#include <os5000/os5000_asio.h>

// System includes.
// ROS includes.
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// Dynamic reconfigure.
#include <dynamic_reconfigure/server.h>
#include <os5000/os5000Config.h>

namespace os5000
{
namespace serial
{
class OS5000
{
 public:
  //! Constructor.
  //! \param nh_ The node handle that topics and parameters are attached to.
  explicit OS5000(ros::NodeHandle nh);

 private:
  //! Establish communications with the compass using Boost ASIO.
  void setup();

  //! Callback function for timer that kicks off all the work.
  void timerCallback(const ros::TimerEvent &event);

  //! Callback function for dynamic reconfigure server.
  void configCallback(os5000::os5000Config &config, uint32_t level);

  //! Return whether we are connected to the compass.
  bool isConnected();

  //! Return roll angle.
  float getRoll();

  //! Return pitch angle.
  float getPitch();

  //! Return yaw angle.
  float getYaw();

  //! Return temperature.
  float getTemperature();

  //! Set yaw angle.
  float setYaw(float difference);

  //! Publish the IMU data.
  void publishImuData();

  //! Simulate data in the case that the compass is not connected.
  void simulateData();

  tf::TransformBroadcaster tf_broadcaster_;
  sensor_msgs::Imu imu_;
  ros::Publisher pub_imu_data_;
  dynamic_reconfigure::Server<os5000::os5000Config> reconfig_srv_;
  dynamic_reconfigure::Server<os5000::os5000Config>::CallbackType reconfig_cb_;
  ros::Timer timer_;
  boost::shared_ptr<asio::OS5000Serial> serial_;

  int rate_;
  float pitch_;
  float roll_;
  float yaw_;
  float temperature_;
  float depth_;
  bool compass_initialized_;
  std::string portname_;
  int baud_;
};
}
}

#endif
