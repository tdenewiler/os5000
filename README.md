# os5000

ROS package for the Ocean Server [OS5000][OS5000] digital compass.

This device gives roll, pitch, and yaw measurements.
Heading is available referenced from magnetic north or true north.

This node outputs a [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) message containing the orientation of the sensor.
The default rate is 40 Hz.
The default topic name is `data`.

## Travis CI

[![Travis-CI](https://api.travis-ci.org/tdenewiler/os5000.svg?branch=master)](https://travis-ci.org/tdenewiler/os5000/branches)

[OS5000]: https://ocean-server.com/wp-content/uploads/2018/08/OS5000_Compass_Manual.pdf
