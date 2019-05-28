#include <os5000/os5000_core.h>
#include <random>

namespace os5000
{
namespace serial
{
OS5000::OS5000(ros::NodeHandle nh)
    : rate_(40),
      pitch_(0.0),
      roll_(0.0),
      yaw_(0.0),
      temperature_(0.0),
      depth_(0.0),
      compass_initialized_(false),
      portname_("/dev/ttyUSB0"),
      baud_(115200)
{
  // Set up a dynamic reconfigure server.
  reconfig_cb_ = boost::bind(&OS5000::configCallback, this, _1, _2);
  reconfig_srv_.setCallback(reconfig_cb_);

  // Initialize node parameters.
  ros::NodeHandle pnh("~");
  pnh.param("baud", baud_, baud_);
  pnh.param("port", portname_, portname_);
  pnh.param("rate", rate_, rate_);
  if (rate_ <= 0)
  {
    rate_ = 1;
  }

  double orientation_covariance = 1.0;
  pnh.param("orientation_covariance", orientation_covariance, orientation_covariance);

  // Create a publisher.
  pub_imu_data_ = pnh.advertise<sensor_msgs::Imu>("data", 1);

  // Create a timer.
  timer_ = nh.createTimer(ros::Duration(1.0 / rate_), &OS5000::timerCallback, this);

  // Set up the compass.
  setup();

  if (!isConnected())
  {
    ROS_ERROR(
        "Could not connect to compass on port %s at %d baud. You can try changing the parameters using the"
        "dynamic reconfigure gui.",
        portname_.c_str(), baud_);
  }

  // Set up message data. Using -1.0 on diagonal to indicate specific message field is invalid.
  double linear_acceleration_covariance = -1.0;
  double angular_velocity_covariance = -1.0;

  imu_.orientation_covariance[0] = orientation_covariance;
  imu_.orientation_covariance[4] = orientation_covariance;
  imu_.orientation_covariance[8] = orientation_covariance;

  imu_.angular_velocity_covariance[0] = angular_velocity_covariance;
  imu_.angular_velocity_covariance[4] = angular_velocity_covariance;
  imu_.angular_velocity_covariance[8] = angular_velocity_covariance;

  imu_.linear_acceleration_covariance[0] = linear_acceleration_covariance;
  imu_.linear_acceleration_covariance[4] = linear_acceleration_covariance;
  imu_.linear_acceleration_covariance[8] = linear_acceleration_covariance;
}

// Ignore unused parameters since the callback signature cannot be changed by us.
void OS5000::timerCallback(const ros::TimerEvent &event)  // NOLINT
{
  serial_->getValues(&roll_, &pitch_, &yaw_, &temperature_);
  ROS_DEBUG("Got (%0.1f, %0.1f, %0.1f)", roll_, pitch_, yaw_);

  // Publish the message.
  publishImuData();
}

void OS5000::simulateData()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> dis(0.0, 0.1);

  pitch_ = pitch_ + dis(gen);
  roll_ = roll_ + dis(gen);
  yaw_ = yaw_ + dis(gen);
  depth_ = depth_ + dis(gen);
  temperature_ = 0 + dis(gen);
}

void OS5000::publishImuData()
{
  geometry_msgs::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";

  // Convert from NED to NWU in a simple way because no rates or accelerations are provided by this compass.
  // Otherwise, a rotation matrix would be the way to go to keep everything consistent.
  pitch_ *= -1.;
  yaw_ *= -1.;

  if (!isConnected())
  {
    simulateData();
  }

  imu_.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(roll_ * M_PI / 180., pitch_ * M_PI / 180., yaw_ * M_PI / 180.);

  ROS_DEBUG("RPY = %0.2lf, %0.2lf, %0.2lf", roll_, pitch_, yaw_);

  pub_imu_data_.publish(imu_);

  // Update transform for sensor state.
  transform.header.stamp = ros::Time::now();
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = imu_.orientation.x;
  transform.transform.rotation.y = imu_.orientation.y;
  transform.transform.rotation.z = imu_.orientation.z;
  transform.transform.rotation.w = imu_.orientation.w;

  // Send the transform describing current sensor state.
  tf_broadcaster_.sendTransform(transform);
}

// Ignore unused parameters since the callback signature cannot be changed by us.
void OS5000::configCallback(os5000::os5000Config &config, uint32_t level)  // NOLINT
{
  ROS_DEBUG("Reconfiguring port, baud, rate, init_time, reconnect to %s, %d, %d, %d, %d", config.port.c_str(),
            config.baud, config.rate, config.init_time, config.reconnect);

  // Set class variables to new values.
  baud_ = config.baud;
  portname_ = config.port;
  rate_ = config.rate;
  if (!isConnected())
  {
    roll_ = config.roll_sim;
    pitch_ = config.pitch_sim;
    yaw_ = config.yaw_sim;
  }

  // Check to see if we should attempt to reconnect to the compass.
  if (config.reconnect)
  {
    // Use the new compass settings to reconnect.
    setup();

    // Reset the reconnect variable.
    config.reconnect = false;
  }
}

bool OS5000::isConnected()
{
  return compass_initialized_;
}

void OS5000::setup()
{
  serial_.reset(new asio::OS5000Serial());
  if (serial_->connect(portname_, baud_))
  {
    ROS_INFO("Initialized compass.");
    compass_initialized_ = true;
  }

  serial_->init(rate_);
}
}
}
