#include <os5000/os5000_asio.h>

namespace os5000
{
namespace asio
{
OS5000Serial::OS5000Serial() : roll_(0.0), pitch_(0.0), yaw_(0.0), temp_(0.0)
{
}

OS5000Serial::~OS5000Serial()
{
  init(0);
  port_->close();
  io_->stop();
  thread_->join();
  io_.reset();
  thread_.reset();
  work_.reset();
}

bool OS5000Serial::connect(const std::string &port, const int baud)
{
  ROS_INFO("Opening port %s at %d baud", port.c_str(), baud);
  io_.reset(new boost::asio::io_service());
  work_.reset(new boost::asio::io_service::work(*io_));
  thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, io_)));

  port_.reset(new boost::asio::serial_port(*io_, port));
  boost::system::error_code error_code;
  port_->set_option(boost::asio::serial_port::baud_rate(baud), error_code);
  port_->set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  if (port_->is_open())
  {
    port_->close();
  }
  port_->open(port, error_code);
  if (error_code != 0)
  {
    ROS_ERROR_STREAM("Could not open port with ASIO. Error = " << error_code);
    return false;
  }

  if (port_->is_open())
  {
    doRead();
  }

  return port_->is_open();
}

void OS5000Serial::doRead()
{
  boost::asio::streambuf::mutable_buffers_type mutable_buffer = buffer_.prepare(128);
  port_->async_read_some(boost::asio::buffer(mutable_buffer),
                         boost::bind(&OS5000Serial::onRead, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}

void OS5000Serial::onRead(const boost::system::error_code &error, std::size_t size)
{
  boost::mutex::scoped_lock look(mutex_);
  ROS_DEBUG_STREAM("In onRead() with " << size << " bytes and result is " << error.message());
  if (!error)
  {
    doRead();
  }

  boost::asio::streambuf::const_buffers_type bufs = buffer_.data();
  std::string str(boost::asio::buffers_begin(bufs), boost::asio::buffers_begin(bufs) + size);
  data_.append(str);
  findMsg();
}

void OS5000Serial::init(const int rate)
{
  std::vector<std::string> cmd;
  cmd.push_back("\x1b");
  cmd.push_back("R");
  cmd.push_back(boost::lexical_cast<std::string>(rate));
  cmd.push_back("\r");
  for (std::vector<std::string>::const_iterator i = cmd.begin(); i != cmd.end(); ++i)
  {
    send(*i);
    usleep(5000);
  }
}

void OS5000Serial::send(const std::string &cmd)
{
  if (port_->is_open())
  {
    boost::system::error_code error_code;
    size_t bytes_sent = port_->write_some(boost::asio::buffer(cmd), error_code);
  }
}

void OS5000Serial::findMsg()
{
  std::string checksum;
  size_t start = data_.find('$');
  if (start == std::string::npos)
  {
    return;
  }
  data_.erase(0, start);

  size_t end = data_.find('\r');
  if (end == std::string::npos)
  {
    return;
  }

  std::string msg;
  try
  {
    msg = data_.substr(start, end);
  }
  catch (const std::out_of_range &e)
  {
    return;
  }
  data_.erase(start, end);

  start = msg.find('C');
  end = msg.find('P');
  if (start == std::string::npos || end == std::string::npos)
  {
    return;
  }
  try
  {
    yaw_ = boost::lexical_cast<float>(msg.substr(start + 1, end - start - 1));
  }
  catch (boost::bad_lexical_cast &e)
  {
    return;
  }

  start = end;
  end = msg.find('R');
  if (end == std::string::npos)
  {
    return;
  }
  try
  {
    pitch_ = boost::lexical_cast<float>(msg.substr(start + 1, end - start - 1));
  }
  catch (boost::bad_lexical_cast &e)
  {
    return;
  }

  start = end;
  end = msg.find('T');
  if (end == std::string::npos)
  {
    return;
  }
  try
  {
    roll_ = boost::lexical_cast<float>(msg.substr(start + 1, end - start - 1));
  }
  catch (boost::bad_lexical_cast &e)
  {
    return;
  }

  start = end;
  end = msg.find('*');
  if (end == std::string::npos)
  {
    return;
  }
  try
  {
    temp_ = boost::lexical_cast<float>(msg.substr(start + 1, end - start - 1));
  }
  catch (boost::bad_lexical_cast &e)
  {
    return;
  }

  if (msg.size() < end + 1)
  {
    return;
  }
  checksum = msg.erase(0, end + 1);

  ROS_DEBUG("Found (%0.1f, %0.1f, %0.1f, %0.1f, %s)", roll_, pitch_, yaw_, temp_, checksum.c_str());
}

void OS5000Serial::getValues(float *roll, float *pitch, float *yaw, float *temp)
{
  *roll = roll_;
  *pitch = pitch_;
  *yaw = yaw_;
  *temp = temp_;
}
}
}
