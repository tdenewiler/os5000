#ifndef SR_SERIAL_H
#define SR_SERIAL_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <string>
#include <string.h>
#include <iostream>

/** @name Maximum number of serial ports to check for available data. */
#ifndef SERIAL_MAX_PORTS
#define SERIAL_MAX_PORTS 10
#endif // SERIAL_MAX_PORTS

/** @name Timeout in [s] to check for available serial ports. */
#ifndef SERIAL_TIMEOUT_SECS
#define SERIAL_TIMEOUT_SECS 0
#endif // SERIAL_TIMEOUT_SECS

/** @name Timeout in [us] to check for available serial ports. */
#ifndef SERIAL_TIMEOUT_USECS
#define SERIAL_TIMEOUT_USECS 1
#endif // SERIAL_TIMEOUT_USECS

/** @name The maximum amount of serial data that can be stored. */
#ifndef SERIAL_MAX_DATA
#define SERIAL_MAX_DATA 65535
#endif // SERIAL_MAX_DATA

#ifndef SERIAL_EXTRA_DELAY_LENGTH
#define SERIAL_EXTRA_DELAY_LENGTH 40000
#endif // SERIAL_EXTRA_DELAY_LENGTH

class Serial
{
public:
    //! Constructor.
    //! \param setup_baud The baud rate that the serial port should be opened at.
    //! \param setup_portname The name of the serial port.
    Serial(std::string _portname, int _baud);

    //! Destructor.
    ~Serial();

    //! Opens a serial port. If SERIAL_DEBUG is defined then open system call
    //! errors are printed to screen.
    virtual void setup();

    //! Writes commands to serial port. If SERIAL_DEBUG is defined then write
    //! system call errors are printed to screen.
    void send();

    //! Get data from the serial port. If SERIAL_DEBUG is defined then read system
    //! call errors are printed to screen.
    void recv();

    //! Checks to see how many bytes are available to read on the serial stack. If
    //! SERIAL_DEBUG is defined then the ioctl system call error is printed to the
    //! screen.
    void getBytesAvailable();

    //! The file descriptor to use for the port. Used when setting up the port
    //! and for sending/receiving data on the port.
    int fd;

    //! The name of the port to open.
    std::string portname;

    //! The baud rate to use with the port.
    int baud;

    //! The length of the message to send.
    int length_send;

    //! The expected length of the message to receive.
    int length_recv;

    //! The data to be sent out of the port.
    char *buf_send;

    //! The data to received on the port.
    char buf_recv[SERIAL_MAX_DATA];

    //! Number of bytes sent.
    int bytes_sent;

    //! Number of bytes received.
    int bytes_recv;

    //! Number of bytes available in serial port buffer.
    int bytes_available;
};

#endif // SR_SERIAL_H
