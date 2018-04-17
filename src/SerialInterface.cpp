/*
 * SerialInterface.cpp
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

#include <francor_levelling_platform/SerialInterface.h>

using namespace boost::asio;

namespace francor
{
    SerialInterface::SerialInterface() :
    _ioService(nullptr), _serialPort(nullptr), _deadlineTimer(nullptr),
    _isInitialized(false)
    {
        for(unsigned int n = 0; n < FRANCOR_SERIAL_INTERFACE_BUFF; n++) {
            this->_asyncReceiveBuffer[n] = 0x00;
        }
        this->_asyncReceiveBufferSize = 0;
    }

    SerialInterface::~SerialInterface() {
        //check if device was released
        if(_isInitialized == true) {
            //no -> release device
            disconnect();
        }
    }

    SerialInterface::Result SerialInterface::connect(const std::string& deviceName, const unsigned int& baudRate) {

        //check if class is already connected
        if(_isInitialized == true)    disconnect();

        //allocate memory
        if(_ioService == nullptr)      _ioService      =   new boost::asio::io_service;
        if(_serialPort == nullptr)     _serialPort     =   new boost::asio::serial_port(*_ioService);
        if(_deadlineTimer == nullptr)  _deadlineTimer  =   new boost::asio::deadline_timer(*_ioService);

        //memory is now allocated
        _isInitialized   =   true;

        //try to open the port
        try {
            //Open the serial port
            _serialPort->open(deviceName);

            //set baudrate
            _serialPort->set_option(serial_port_base::baud_rate(baudRate));

            //set flow control
            serial_port_base::flow_control flowCtrl(serial_port_base::flow_control::none);
            _serialPort->set_option(flowCtrl);

            //set one stop bit
            serial_port_base::stop_bits stopBit(serial_port_base::stop_bits::one);
            serial_port_base::parity parityBit(serial_port_base::parity::none);

            //RS232 8N1
            _serialPort->set_option(serial_port_base::character_size(8)); //8 data bits
            _serialPort->set_option(stopBit); //1 stop bit
            _serialPort->set_option(parityBit); //0 parity bits

        }
        catch(boost::system::system_error& error) {

            //throw error message
            ROS_ERROR("RS232::Port::connect(): boost error: %s", error.what());

            return FAILED;
        }


        return SUCCESS;
    }

    void SerialInterface::disconnect() {
        //check if is initialized
        if(_isInitialized == false) return;

        //close port
        if(_serialPort != NULL) {
            if(_serialPort->is_open()) {
                _serialPort->close();
            }
        }

        //free memory
        if(_deadlineTimer != nullptr) {delete(_deadlineTimer);_deadlineTimer = nullptr;}
        if(_serialPort != nullptr)    {delete(_serialPort);_serialPort = nullptr;}
        if(_ioService != nullptr)     {delete(_ioService);_ioService = nullptr;}

        //connection closed
        _isInitialized    =   false;
    }

    SerialInterface::Result SerialInterface::transmitBuffer(const unsigned char data[], const unsigned int& dataSize) {
        //check if class is initialized
        if(!_isInitialized) {ROS_ERROR("RS232::Port::transmitBuffer(): Not initialized!"); return ERROR_NOT_INITIALIZED;}

        try {
            //transmit data
            _serialPort->write_some(buffer(data, dataSize));
        }
        catch(boost::system::system_error& error) {
            //throw error
            ROS_ERROR("RS232::Port::transmitBuffer(): boost error: %s", error.what());

            return FAILED;
        }

        return SUCCESS;
    }

    SerialInterface::Result SerialInterface::receiveBuffer(unsigned char* data, unsigned int& dataSize, const unsigned int& dataMaxSize, const unsigned int& timeoutMS) {
        //check if class is initialized
        if(!_isInitialized) {ROS_ERROR("RS232::Port::transmitBuffer(): Not initialized!"); return ERROR_NOT_INITIALIZED;}

        Result          result;
        unsigned char   buffer  =   0;
        unsigned int    pos     =   0;

        //dataSize = 0
        dataSize = 0;

        //receive data until escape sequence is reached or maximum amount of data is reached
        while(pos < dataMaxSize) {

            //receive data
            result = receiveChar(buffer, timeoutMS);

            //check if error occured
            if(result != SUCCESS) {
                return result;
            }

            //save data
            data[pos++]   =   buffer;

            //check for escape sequence
            if(pos > 2) {
                if(data[pos - 2] == '\r' && data[pos - 1] == '\n') {
                    dataSize = pos;

                    //receiving finished
                    return SUCCESS;
                }
            }

        }

        //throw error
        ROS_INFO("data: %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i %i",
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11],
                data[12], data[13], data[14], data[15]);
        ROS_ERROR("RS232::Port::receiveBuffer(): Max receive buffer size reached!");

        return ERROR_MAX_SIZE;
    }

    SerialInterface::Result SerialInterface::receiveChar(unsigned char& data, const unsigned int& timeoutMS) {

        unsigned char receiveBuffer[1];
        bool data_available = false;

        //read data asynchronous
        _serialPort->async_read_some( boost::asio::buffer(receiveBuffer),
                                    boost::bind(&SerialInterface::read_callback,
                                    this,
                                    boost::ref(data_available),
                                    boost::ref(*_deadlineTimer),
                                    boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred)
                                    );

        //config timeout
        boost::posix_time::millisec delay_boost(timeoutMS);
        _deadlineTimer->expires_from_now(delay_boost);
        _deadlineTimer->async_wait(boost::bind(&SerialInterface::wait_callback,
                                this,
                                boost::ref(*_serialPort),
                                boost::asio::placeholders::error));

        //block until async callback is finished or timeout occured
        _ioService->run();

        //reset
        _ioService->reset();

        //check if data was received
        if(!data_available) {
            //there was no data received
            ROS_WARN("RS232::Port::receiveChar(): Timeout!");

            return ERROR_TIMEOUT;
        }

        //save received data
        data = receiveBuffer[0];

        return SUCCESS;
    }

    void SerialInterface::read_callback(bool& data_available, boost::asio::deadline_timer& timeout,
                            const boost::system::error_code& error, std::size_t bytes_transferred )
    {
        //std::cout << "debug: called -> SerialUSB::read_callback(...)" << std::endl;
        if (error || !bytes_transferred)
        {
            // No data was read!
            data_available = false;
            return;
        }

        timeout.cancel();  // will cause wait_callback to fire with an error
        data_available = true;
    }

    void SerialInterface::wait_callback(boost::asio::serial_port& ser_port, const boost::system::error_code& error)
    {
        //std::cout << "debug: called -> SerialUSB::wait_callback(...)" << std::endl;
        if(error)
        {
            // Data was read and this timeout was canceled
            return;
        }

        ser_port.cancel();  // will cause read_callback to fire with an error
    }
} /*namespace francor*/
