#include "os5000/os5000_core.h"

namespace os5000
{
namespace serial
{
OS5000::OS5000(ros::NodeHandle nh_)
{
  // Set up a dynamic reconfigure server.
  reconfig_cb_ = boost::bind(&OS5000::configCallback, this, _1, _2);
  reconfig_srv_.setCallback(reconfig_cb_);

  // Initialize node parameters.
  ros::NodeHandle pnh("~");
  pnh.param("baud", baud_, 115200);
  pnh.param("init_time", init_time_, 3);
  pnh.param("port", portname_, std::string("/dev/ttyUSB0"));
  pnh.param("rate", rate_, 40);
  pnh.param("use_asio", use_asio_, true);
  if (rate_ <= 0)
  {
    rate_ = 1;
  }

  // Initialize variables.
  bytes_available_ = 0;
  start_time_ = ros::Time::now();

  // Create a publisher.
  createPublisher(&pnh);

  // Create a timer.
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate_), &OS5000::timerCallback, this);

  // Set up the compass.
  if (use_asio_)
  {
    setupASIO();
  }
  else
  {
    setup();
  }

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
  double orientation_covariance = 1.;

  imudata_.orientation_covariance[0] = orientation_covariance;
  imudata_.orientation_covariance[4] = orientation_covariance;
  imudata_.orientation_covariance[8] = orientation_covariance;

  imudata_.angular_velocity.x = 0.;
  imudata_.angular_velocity.y = 0.;
  imudata_.angular_velocity.z = 0.;

  imudata_.angular_velocity_covariance[0] = angular_velocity_covariance;
  imudata_.angular_velocity_covariance[4] = angular_velocity_covariance;
  imudata_.angular_velocity_covariance[8] = angular_velocity_covariance;

  imudata_.linear_acceleration.x = 0.;
  imudata_.linear_acceleration.y = 0.;
  imudata_.linear_acceleration.z = 0.;

  imudata_.linear_acceleration_covariance[0] = linear_acceleration_covariance;
  imudata_.linear_acceleration_covariance[4] = linear_acceleration_covariance;
  imudata_.linear_acceleration_covariance[8] = linear_acceleration_covariance;
}

OS5000::~OS5000()
{
  // Close the open file descriptors.
  if (fd_ > 0)
  {
    close(fd_);
  }
}

void OS5000::run()
{
  ros::spin();
}

void OS5000::setup()
{
  // Setup the serial port.
  serialSetup();

  // Declare variables.
  compass_initialized_ = false;

  // Check if the compass serial port was opened correctly.
  if (fd_ < 0)
  {
    // Seed the random number generator to use when simulating data.
    srand(static_cast<unsigned int>(time(NULL)));

    return;
  }

  // Set the message rate to control how often the compass will send out updates.
  setRate();

  // Initialize the timer.
  start_time_ = ros::Time::now();

  // Check for valid compass data until the timer expires or valid data is found.
  while (ros::Time::now().toSec() - start_time_.toSec() < init_time_)
  {
    // Try to initialize the compass to make sure that we are actually getting valid data from it.
    init();
    if (compass_initialized_)
    {
      break;
    }
  }

  // No valid compass data found so simulate compass data in the main loop.
  if (!compass_initialized_)
  {
    // Seed the random number generator to use when simulating data.
    srand(static_cast<unsigned int>(time(NULL)));
    fd_ = -1;
  }
}

void OS5000::getData()
{
  // Declare variables.
  int bytes_to_discard = 0;
  found_complete_message_ = false;

  // Get the number of bytes available on the serial port.
  getBytesAvailable();

  // Make sure we don't read too many bytes and overrun the buffer.
  if (bytes_available_ >= SERIAL_MAX_DATA)
  {
    bytes_available_ = SERIAL_MAX_DATA - 1;
  }
  if (bytes_available_ + strlen(buf_recv_) >= SERIAL_MAX_DATA)
  {
    bytes_to_discard = bytes_available_ + strlen(buf_recv_) - SERIAL_MAX_DATA - 1;
    memmove(buf_recv_, &buf_recv_[bytes_to_discard], bytes_to_discard);
  }

  // Read data off the serial port.
  if (bytes_available_ > 0)
  {
    recv();
    // Look for entire message.
    findMsg();
    if (found_complete_message_)
    {
      // Parse the message.
      parseMsg();
    }
  }
}

void OS5000::timerCallback(const ros::TimerEvent &event)
{
  if (use_asio_)
  {
    float temp = 0.0;
    serial_->getValues(&roll_, &pitch_, &yaw_, &temp);
    ROS_DEBUG("Got (%0.1f, %0.1f, %0.1f)", roll_, pitch_, yaw_);
  }
  else
  {
    if (isConnected())
    {
      imudata_.header.stamp = ros::Time::now();
      getData();

      float current_yaw = getYaw();
      if (current_yaw > 180.)
      {
        current_yaw -= 360.;
        setYaw(current_yaw);
      }
    }
    else
    {
      // Simulate data.
      // roll = 0.0;
      // pitch = 0.0;
      // yaw = 0.0;
    }
  }

  // Publish the message.
  publishImuData();
}

void OS5000::setRate()
{
  // Declare variables.
  char cmd = 0;
  OS5000::buf_send_ = &cmd;

  // send command.
  if (fd_ > 0)
  {
    // send out escape and R commands.
    cmd = OS5000_CMD_ESC;
    length_send_ = 1;
    ROS_INFO_STREAM("Sending cmd = " << cmd);
    send();
    usleep(OS5000_SERIAL_DELAY);
    cmd = OS5000_CMD_SET_RATE;
    length_send_ = 1;
    ROS_INFO_STREAM("Sending cmd = " << cmd);
    send();
    usleep(OS5000_SERIAL_DELAY);

    // Compute the rate command.
    if (rate_ <= 0)
    {
      cmd = '0';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    else if (rate_ >= 40)
    {
      cmd = '4';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
      cmd = '0';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    else if (rate_ >= 30)
    {
      cmd = '3';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
      cmd = '0';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    else if (rate_ >= 20)
    {
      cmd = '2';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
      cmd = '0';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    else if (rate_ >= 10)
    {
      cmd = '1';
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
      cmd = rate_ + 38;
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    else
    {
      cmd = rate_ + 48;
      length_send_ = 1;
      ROS_INFO_STREAM("Sending cmd = " << cmd);
      send();
      usleep(OS5000_SERIAL_DELAY);
    }
    cmd = OS5000_CMD_END;
    length_send_ = 1;
    ROS_INFO_STREAM("Sending cmd = " << cmd);
    send();
    usleep(OS5000_SERIAL_DELAY);
  }
}

void OS5000::zeroDepth()
{
  // Declare variables.
  char cmd = 0;
  OS5000::buf_send_ = &cmd;

  // send command.
  if (fd_ > 0)
  {
    // send out escape and R commands.
    cmd = OS5000_CMD_ESC;
    length_send_ = 1;
    send();
    usleep(OS5000_SERIAL_DELAY);

    // send out zero depth command.
    cmd = OS5000_CMD_ZERO_DEPTH;
    length_send_ = 1;
    send();
    usleep(OS5000_SERIAL_DELAY);

    // send out the ending command.
    cmd = OS5000_CMD_END;
    length_send_ = 1;
    send();
    usleep(OS5000_SERIAL_DELAY);
  }
}

void OS5000::findMsg()
{
  // Declare variables.
  bool found_start = false;
  int i = 0;
  int msg_start_location = 0;
  found_complete_message_ = false;

  // Look for start character.
  for (i = 0; i < length_recv_; i++)
  {
    if (found_start)
    {
      if (OS5000::buf_recv_[i] == OS5000_RESPONSE_END)
      {
        found_complete_message_ = true;
        found_start = false;
        // Place the last complete message at the beginning of the buffer and throw away any data before that.
        memmove(OS5000::buf_recv_, OS5000::buf_recv_ + msg_start_location, length_recv_ - msg_start_location);
      }
    }
    // Look for start sequence. Don't assume that there is an end response before another start response.
    if (OS5000::buf_recv_[i] == OS5000_RESPONSE_START)
    {
      found_start = true;
      msg_start_location = i;
    }
  }
}

void OS5000::parseMsg()
{
  // Declare variables.
  int i = 0;
  char *token;
  char *saveptr;
  char const *delim = reinterpret_cast<char const *>(OS5000_DELIM);
  char *tokens[OS5000_STRING_SIZE];

  // Initialize variables.
  memset(tokens, 0, sizeof(tokens));

  // Split the incoming buffer up into tokens based on the delimiting characters.
  tokens[0] = strtok_r(OS5000::buf_recv_, delim, &saveptr);
  for (i = 1; i < length_recv_; i++)
  {
    token = strtok_r(NULL, delim, &saveptr);
    if (token == NULL)
    {
      break;
    }
    tokens[i] = token;
  }

  // Store Euler angles, temperature, depth values.
  if (tokens[0] != NULL)
  {
    yaw_ = atof(tokens[0]);
  }
  if (tokens[1] != NULL)
  {
    pitch_ = atof(tokens[1]);
  }
  if (tokens[2] != NULL)
  {
    roll_ = atof(tokens[2]);
  }
  if (tokens[3] != NULL)
  {
    temperature_ = atof(tokens[3]);
  }
  if (tokens[4] != NULL)
  {
    depth_ = atof(tokens[4]);
  }
}

void OS5000::simulateData()
{
  pitch_ = pitch_ + rand() / static_cast<float>(RAND_MAX);
  roll_ = roll_ + rand() / static_cast<float>(RAND_MAX);
  yaw_ = yaw_ + rand() / static_cast<float>(RAND_MAX);
  temperature_ = 0 + rand() / static_cast<float>(RAND_MAX);
  depth_ = depth_ + rand() / static_cast<float>(RAND_MAX);
}

void OS5000::init()
{
  // Initialize variables.
  compass_initialized_ = false;

  // Get the number of bytes available on the serial port.
  tcflush(fd_, TCIFLUSH);
  usleep(OS5000_SERIAL_DELAY);
  getBytesAvailable();

  // Read data off the serial port.
  if (bytes_available_ > 0)
  {
    length_recv_ = bytes_available_;
    recv();
    // Look for entire message.
    findMsg();
    if (found_complete_message_)
    {
      // Parse the message.
      parseMsg();
      compass_initialized_ = true;
    }
  }
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

  imudata_.orientation =
      tf::createQuaternionMsgFromRollPitchYaw(roll_ * M_PI / 180., pitch_ * M_PI / 180., yaw_ * M_PI / 180.);

  ROS_DEBUG("OS5000 Quaternions = %.1f, %.1f, %.1f, %.1f", imudata_.orientation.x, imudata_.orientation.y,
            imudata_.orientation.z, imudata_.orientation.w);
  ROS_DEBUG("OS5000 (RPY) = (%lf, %lf, %lf)", roll_, pitch_, yaw_);

  pub_imu_data_.publish(imudata_);

  // Update transform for sensor state.
  transform.header.stamp = ros::Time::now();
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation.x = imudata_.orientation.x;
  transform.transform.rotation.y = imudata_.orientation.y;
  transform.transform.rotation.z = imudata_.orientation.z;
  transform.transform.rotation.w = imudata_.orientation.w;

  // Send the transform describing current sensor state.
  tf_broadcaster_.sendTransform(transform);
}

void OS5000::configCallback(os5000::os5000Config &config, uint32_t level)
{
  ROS_DEBUG("Reconfiguring port, baud, rate, init_time, reconnect to %s, %d, %d, %d, %d", config.port.c_str(),
            config.baud, config.rate, config.init_time, config.reconnect);

  // Set class variables to new values.
  baud_ = config.baud;
  init_time_ = config.init_time;
  portname_ = config.port.c_str();
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
    ROS_INFO("Using new settings to reconnect to compass. Got fd = %d", fd_);

    // Reset the reconnect variable.
    config.reconnect = false;
  }
}

float OS5000::getRoll()
{
  return roll_;
}

float OS5000::getPitch()
{
  return pitch_;
}

float OS5000::getYaw()
{
  return yaw_;
}

float OS5000::setYaw(float difference)
{
  return (yaw_ + difference);
}

float OS5000::getTemperature()
{
  return temperature_;
}

bool OS5000::isConnected()
{
  return compass_initialized_;
}

void OS5000::createPublisher(ros::NodeHandle *pnh)
{
  pub_imu_data_ = pnh->advertise<sensor_msgs::Imu>("data", 1);
}

void OS5000::serialSetup()
{
  // Declare variables.
  int status = -1;
  struct termios options;

  // Open port.
  fd_ = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 1)
  {
    ROS_ERROR("Failed to open port %s: %s", portname_.c_str(), strerror(errno));
    return;
  }

  // Get the current options for the port.
  tcgetattr(fd_, &options);

  // Set the baud rate.
  switch (baud_)
  {
    case 1200:
      cfsetispeed(&options, B1200);
      cfsetospeed(&options, B1200);
      break;

    case 2400:
      cfsetispeed(&options, B2400);
      cfsetospeed(&options, B2400);
      break;

    case 4800:
      cfsetispeed(&options, B4800);
      cfsetospeed(&options, B4800);
      break;

    case 9600:
      cfsetispeed(&options, B9600);
      cfsetospeed(&options, B9600);
      break;

    case 19200:
      cfsetispeed(&options, B19200);
      cfsetospeed(&options, B19200);
      break;

    case 38400:
      cfsetispeed(&options, B38400);
      cfsetospeed(&options, B38400);
      break;

    case 57600:
      cfsetispeed(&options, B57600);
      cfsetospeed(&options, B57600);
      break;

    case 115200:
      cfsetispeed(&options, B115200);
      cfsetospeed(&options, B115200);
      break;

    case 230400:
      cfsetispeed(&options, B230400);
      cfsetospeed(&options, B230400);
      break;

    default:  // Bad baud rate passed.
      close(fd_);
      fd_ = -2;
      ROS_ERROR("Bad baud rate passed.");
      return;
  }

  // Set the number of data bits.
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // Set the number of stop bits to 1.
  options.c_cflag &= ~CSTOPB;

  // Set parity to none.
  options.c_cflag &= ~PARENB;

  // Set for non-canonical (raw processing, no echo, etc.).
  options.c_iflag = IGNPAR;  // Ignore parity check.
  options.c_oflag = 0;       // Raw output.
  options.c_lflag = 0;       // Raw input.

  // Time-Outs -- won't work with NDELAY option in the call to open.
  options.c_cc[VMIN] = 0;    // Block reading until RX x characers. If x = 0, it is non-blocking.
  options.c_cc[VTIME] = 10;  // Inter-Character Timer -- i.e. timeout= x*.1 s.

  // Set local mode and enable the receiver.
  options.c_cflag |= (CLOCAL | CREAD);

  // Purge the serial port buffer.
  tcflush(fd_, TCIFLUSH);

  // Set the new options for the port.
  status = tcsetattr(fd_, TCSANOW, &options);
  if (status != 0)
  {
    ROS_ERROR("Failed to set up serial port options");
    fd_ = -3;
    return;
  }
}

void OS5000::recv()
{
  // Declare variables.
  int port_count = 0;
  int bytes_recv = 0;
  struct timeval timeout;
  fd_set serial_fds;
  memset(&buf_recv_, 0, sizeof(buf_recv_));

  // Set the timeout values for receiving data so the port doesn't block.
  timeout.tv_sec = SERIAL_TIMEOUT_SECS;
  timeout.tv_usec = SERIAL_TIMEOUT_USECS;

  // Add the current port to the list of serial ports.
  FD_ZERO(&serial_fds);
  FD_SET(fd_, &serial_fds);

  // Set up the port so it can be read from.
  port_count = select(SERIAL_MAX_PORTS, &serial_fds, NULL, NULL, &timeout);
  if ((port_count == 0) || (!FD_ISSET(fd_, &serial_fds)))
  {
    bytes_recv = -1;
  }

  // Read the data from the port.
  bytes_recv = read(fd_, buf_recv_, length_recv_);
  if (bytes_recv == -1)
  {
    ROS_WARN("Could not read data from serial port: %s", strerror(errno));
  }
}

void OS5000::send()
{
  // Send data out on the serial port.
  int bytes_sent = write(fd_, buf_send_, length_send_);

  // Flush the data in the port buffer.
  tcdrain(fd_);

  if (bytes_sent < 0)
  {
    ROS_WARN("Could not write data to serial port: %s", strerror(errno));
  }
}

void OS5000::getBytesAvailable()
{
  // Get the number of bytes in the serial port buffer.
  if (ioctl(fd_, FIONREAD, &bytes_available_) == -1)
  {
    ROS_WARN("Could not get number of bytes available on serial port: %s", strerror(errno));
  }
}

void OS5000::setupASIO()
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
