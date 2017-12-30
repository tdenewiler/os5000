#ifndef OS5000_OS5000_ASIO_H
#define OS5000_OS5000_ASIO_H

#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <ros/console.h>

namespace os5000
{
namespace asio
{
class OS5000Serial
{
 public:
  OS5000Serial();

  ~OS5000Serial();

  bool connect(const std::string &port, int baud);

  void init(int rate);

  void getValues(float *roll, float *pitch, float *yaw, float *temp);

 private:
  void send(const std::string &cmd);

  void onRead(const boost::system::error_code &error, std::size_t size);

  void doRead();

  void findMsg();

  boost::shared_ptr<boost::asio::io_service> io_;

  boost::shared_ptr<boost::asio::io_service::work> work_;

  boost::shared_ptr<boost::asio::serial_port> port_;

  boost::shared_ptr<boost::thread> thread_;

  boost::asio::streambuf buffer_;

  std::string data_;

  boost::mutex mutex_;

  float roll_;

  float pitch_;

  float yaw_;

  float temp_;
};
}
}

#endif
