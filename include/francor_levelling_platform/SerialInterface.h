/*
 * SerialInterface.h
 *
 *  Created on: 17.04.2018
 *      Author: feesmrt
 *
 * BSD 3-Clause License
 * Copyright (c) 2018, FRANC0R - Franconian Open Robotics
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef SERIAL_INTERFACE_H_
#define SERIAL_INTERFACE_H_

/**
 * @brief Definitions
 */
#define FRANCOR_SERIAL_INTERFACE_BUFF 256

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>

#include <boost/asio.hpp>
#include <boost/system/system_error.hpp>
#include <boost/bind.hpp>

/**
 * Namespace of francor
 */
namespace francor {

/**
 * Interface to a serial port of the PC.
 */
class SerialInterface
{
public:
    /**
     * Enumeration of Results
     */
    enum Result     {FAILED = 0,               //!< Failure
                     SUCCESS = 1,              //!< Success
                     ERROR_NOT_INITIALIZED = 2,//!< Class is not initialized
                     ERROR_TIMEOUT = 3,        //!< Timeout happened
                     ERROR_MAX_SIZE = 4};      //!< Maximum array size exceeded


    /**
     * Creates the class
     */
     SerialInterface();

    /**
     * Desctructor.
     * Releases closes the connection and releases all allocated memory.
     */
    ~SerialInterface();

    /**
     * Initializes the connection via the serial port
     *
     * @param deviceName[in] Name of the serial port
     * @param baudRate[in] Communication speed
     *
     * @return SUCCESS when connection was established successfully
     */
    Result connect(const std::string& deviceName, const unsigned int& baudRate);

    /**
     * Closes the port and releases all allocated memory
     */
    void disconnect();

    /**
     * Transmits the data buffer via RS232 protocol
     *
     * @param data[in] Array containing the data to send
     * @param dataSize[in] Size of the array
     *
     * @return SUCCESS if data was sent successfully
     */
    Result transmitBuffer(const unsigned char data[], const unsigned int& dataSize);

    /**
     * Receives data via the RS232 protocol.
     * Receiving is finished when escape sequence was received "\r\n"
     *
     * @param data[out] Array with the received data
     * @param dataSize[out] Size of the array
     * @param dataMaxSize[in] Maximum size of the array
     * @param timeoutMS[in] Maximum time to wait for a byte to receive
     *
     * @return SUCCESS if data was received successfully
     */
    Result receiveBuffer(unsigned char* data, unsigned int& dataSize, const unsigned int& dataMaxSize, const unsigned int& timeoutMS);



private:

    /**
     * Receives one byte/char
     * Quits when timeout time is exceeded
     *
     * @param data[out] Byte/Char received
     * @param timeoutMS[in] Maximum time in milliseconds to wait for data
     *
     * @return Success if data was received successfully
     */
    Result receiveChar(unsigned char& data, const unsigned int& timeoutMS);

    /**
     * Called from boost
     *
     * @param data_available
     * @param timeout
     * @param error
     * @param bytes_transferred
     */
    void read_callback(bool& data_available, boost::asio::deadline_timer& timeout,
                       const boost::system::error_code& error, std::size_t bytes_transferred );

    /**
     * Called from boost
     *
     * @param ser_port
     * @param error
     */
    void wait_callback(boost::asio::serial_port& ser_port, const boost::system::error_code& error);


    boost::asio::io_service*        _ioService;                                            /**< Instance of boost I/O service */
    boost::asio::serial_port*       _serialPort;                                           /**< Instance of boost serial port */
    boost::asio::deadline_timer*    _deadlineTimer;                                        /**< Instance of boost timer */

    unsigned char                   _asyncReceiveBuffer[FRANCOR_SERIAL_INTERFACE_BUFF];    /**< Buffer for async receiving data */
    unsigned int                    _asyncReceiveBufferSize;                               /**< Saves the size of the received data */

    bool                            _isInitialized;         /**< Saves true when class is initialized */
};

}
#endif /* SERIAL_INTERFACE_H_ */
