#ifndef OS5000_CORE_H
#define OS5000_CORE_H

// System includes.
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// ROS includes.
#include <ros/ros.h>

// Local includes.
#include "os5000/serial.h"
#include "os5000/timing.h"

/* Command Set Summary for the Ocean Server compass. */
#ifndef OS5000_COMMANDS
#define OS5000_COMMANDS
/** @name Command values for the compass. */
//@{
#define OS5000_CMD_ESC             '\x1b'
#define OS5000_CMD_SET_BAUD        'B'
#define OS5000_CMD_SET_RATE        'R'
#define OS5000_CMD_SET_FIELDS      'X'
#define OS5000_CMD_GET_FIRMWARE    'V'
#define OS5000_CMD_SET_ORIENTATION 'E'
#define OS5000_CMD_ZERO_DEPTH	   'P'
#define OS5000_CMD_GET_CONFIG      '&'
#define OS5000_CMD_END             '\r'
//@}
#endif // OS5000_COMMANDS

#ifndef OS5000_DELIM
#define OS5000_DELIM " $CPRTD*\n\r"
#endif // OS5000_DELIM

#ifndef OS5000_STRING_SIZE
#define OS5000_STRING_SIZE 128
#endif // OS5000_STRING_SIZE

#ifndef OS5000_SERIAL_DELAY
#define OS5000_SERIAL_DELAY 100000
#endif // OS5000_SERIAL_DELAY
//

#ifndef OS5000_RESPONSE
#define OS5000_RESPONSE_START  '$'
#define OS5000_RESPONSE_END    '*'
#endif // OS5000_RESPONSE

#ifndef OS5000_ERROR_HEADER
#define OS5000_SUCCESS	        1
#define OS5000_ERROR_HEADER    -1
#define OS5000_ERROR_CHECKSUM  -2
#define OS5000_ERROR_LENGTH    -3
#define OS5000_ERROR_VALID_MSG -4
#endif // OS5000_ERROR_HEADER

class Compass : public Serial
{
public:
    //! Constructor.
    //! \param _portname The name of the port that the compass is connected to.
    //! \param _baud The baud rate that the compass is configured to communicate at.
    //! \param _rate The rate that the compass should be set up to send messages at.
    //! \param _initTime The amount of time to try establishing communications with the compass before timing out.
    Compass(std::string _portname, int _baud, int _rate, int _initTime);

    //! Destructor.
    ~Compass();

    //! Establish communications with the compass.
    void setup();

    //! Get orientation data from compass.
    void getData();

    //! Set the rate at which messages are sent by the compass.
    //! Valid rates are from 0 - 20. Any numbers outside that range will be shifted
    //! into that range.
    void setRate();
    
	//! Zero the depth pressure sensor. This would be at the surface for an
	//! underwater device. It set’s zero based on the current ATM pressure.
	void zeroDepth();

    //! Simulates compass data when no compass is available.
    void simulateData();

    //! Looks for valid data from the compass for a specified amount of time.
    void init();

    //! Serial number of the compass.
    int serial_number;

    //! Pitch angle in units of degrees, range of (-180,180].
    float pitch;

    //! Roll angle in units of degrees, range of (-180,180].
    float roll;

    //! Yaw angle in units of degrees, range of (-180,180].
    float yaw;

    //! Temperature as measured on the compass board in units of Fahrenheit. This is not a precise measurement but does
    //! give a rough indication of the temperature.
    float temperature;
    
    //! Depth in the form of pressure as measured from the pressure sensor and converted from analog
    //! to digital through the compass.
    float depth;

    //! The file descriptor used to communicate with the compass.
    int fd;

    //! The amount of time attempting to set up communications with the compass.
    int init_time;

    //! The rate that the compass should send out data updates.
    int rate;

    //! Pointer to a timer.
    Timing *timer;

private:
    //! Searches a buffer looking for the start and end sequences.
    void findMsg();

    //! Parses a message for compass data.
    void parseMsg();

    //! Whether the compass was initialized correctly.
    bool compass_initialized;

    //! Whether a complete message was found after reading compass data.
    bool found_complete_message;
};

#endif // OS5000_CORE_H
