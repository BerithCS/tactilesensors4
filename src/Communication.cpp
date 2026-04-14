#include "Communication.h"
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>
#include <string>
#include <algorithm>
#include <iostream>
#include <cstring>

/****************************************************************************************
//  Communication::Communication()
//  Brief:  Communication construtor tries to open the specified port to the sensor. If
//          successful, create 2 finger object and automatically start communication
//          thread with the specified.
//  Param:
//        -deviceName:    the name of the sensor (ex: /dev/ttyACM0)
****************************************************************************************/
Communication::Communication(rclcpp::Node::SharedPtr node, const std::string* deviceName):
    USB(-1),
    isEnabled(true)
{
    if(OpenAndConfigurePort(&USB,deviceName->c_str()))
    {
        fingers.emplace_back(std::make_unique<Finger>(node));
        fingers.emplace_back(std::make_unique<Finger>(node));

        comThread = std::thread(&Communication::run, this);
    }
    else
    {
        //TODO: error
    }
}

/****************************************************************************************
//  Communication::~Communication()
//  Brief:  Stops comm thread before exit
****************************************************************************************/
Communication::~Communication()
{
    stopThread = true;
    if (comThread.joinable())
    {
        enableCondition.notify_all();
        comThread.join();
    }
}

/****************************************************************************************
//  Communication::enableComm(bool enable)
//  Brief:  Use to pause the comm thread
//  Param:
//      -enable: enable or not the comm
****************************************************************************************/
void Communication::enableComm(bool enable)
{
    isEnabled = enable;
    if(isEnabled)
    {
        enableCondition.notify_all();
    }
}

/****************************************************************************************
//  Communication::run()
//  Brief:  Main thread of the Communication object. Use to read the packet streamed by
//          the sensor. When the packet are parsed sucessfully, send the new info the
//          the respective finger object.
****************************************************************************************/
void Communication::run()
{
    stopThread = false;

    UsbPacket send{}, recv{};
    send.command = USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = READ_DATA_PERIOD_MS;
    usbSend(&send);

    int n_read;
    unsigned int recvSoFar = 0;
    std::vector<char> receiveBuffer(4096);

    std::unique_lock<std::mutex> lock(enableMutex);

    //Now we enter the ros loop where we will acquire and publish data
    while (!stopThread)
    {
        if(!isEnabled)
        {
            enableCondition.wait(lock);
            continue;
        }

        //Read the answer, i.e. the bytes received from the sensor:
        n_read = read(USB, receiveBuffer.data(), receiveBuffer.size());

        //   Parse packets and store sensor values
        for (int64_t i = 0; i < n_read; ++i)
        {
            if (usbReadByte(&recv, &recvSoFar, receiveBuffer[i]))
            {
                parseSensors(&recv);
            }
        }
    }

    // Stop auto-send message
    send.command = USB_COMMAND_AUTOSEND_SENSORS;
    send.data_length = 1;
    send.data[0] = 0;
    usbSend(&send);

    close(USB);  // close and free /dev/ttyACM0 peripheral
}

/****************************************************************************************
//Function: OpenAndConfigurePort
//
//Description:  This function opens the /dev/ttyUSB0 port with read and write access
//              rights (that suppose that the user has done "sudo chmod 777 /dev/ttyUSB0"
//              before. If it succeeds, it returns true, otherwise it returns false.
//
//Arguments:    int *USB: A pointer thats points file descriptor attached to ttyUSB0
//                          device.
//              char const * TheDevice: The string of the device we want to open.
//                          (e.g. /dev/ttyACM0).
****************************************************************************************/
bool Communication::OpenAndConfigurePort(int *USB, char const * TheDevice)
{
    // All written by Jean-Philippe Roberge on April 2015, reviewed by Jean-Philippe Roberge in June 2016
    /* Open File Descriptor */
    //            *USB = open( "/tmp/interceptty", O_RDWR| O_NOCTTY ); // For debugging purposes (JP)
    *USB = open(TheDevice, O_RDWR| O_NOCTTY );
    /* Error Handling */
    if ( (*USB) < 0 )
    {
        std::cout << "Error " << errno << " opening " << TheDevice << ": " << std::strerror (errno) << std::endl;
        return false;
    }

    /**** Configure Port ****/
    struct termios tty{};
    memset(&tty, 0, sizeof tty);
    if ( tcgetattr ( (*USB), &tty ) != 0 ) {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
        return false;
    }

    /* Set Baud Rate */
    cfsetospeed (&tty, (speed_t)B115200);
    cfsetispeed (&tty, (speed_t)B115200);

    /* Setting other Port Stuff */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush( (*USB), TCIFLUSH );
    if ( tcsetattr ( (*USB), TCSANOW, &tty ) != 0) {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        return false;
    }
    return true;
}

void Communication::usbSend(UsbPacket *packet) const
{
    auto *p = (uint8_t *)packet;
    int n_written;

    packet->start_byte = USB_PACKET_START_BYTE;
    packet->crc8 = calcCrc8(p + 2, packet->data_length + 2);

    n_written = write( USB, (char *)p, packet->data_length + 4 );
}

uint8_t Communication::calcCrc8(uint8_t *data, size_t len)
{
    // TODO: calculate CRC8
    return data[-1];
}

bool Communication::usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d)
{
    auto *p = (uint8_t *)packet;

    // Make sure start byte is seen
    if (*readSoFar == 0 && d != USB_PACKET_START_BYTE)
        return false;

    // Buffer the byte (making sure not to overflow the packet)
    if (*readSoFar < 64)
        p[*readSoFar] = d;
    ++*readSoFar;

    // If length is read, stop when done
    if (*readSoFar > 3 && *readSoFar >= (unsigned)packet->data_length + 4)
    {
        *readSoFar = 0;

        // If CRC is ok, we have a new packet!  Return it.
        if (packet->crc8 == calcCrc8(p + 2, packet->data_length + 2))
            return true;

        // If CRC is not ok, find the next start byte and shift the packet back in hopes of getting back in sync
        for (unsigned int i = 1; i < (unsigned)packet->data_length + 4; ++i)
            if (p[i] == USB_PACKET_START_BYTE)
            {
                memmove(p, p + i, packet->data_length + 4 - i);
                *readSoFar = packet->data_length + 4 - i;
                break;
            }
    }

    return false;
}


void Communication::parseSensors(UsbPacket *packet)
{
    bool errorFlag = false;

    for (int i = 0; i < packet->data_length;)
    {
        uint8_t sensorType = packet->data[i] & 0xF0;
        uint8_t f = packet->data[i] >> 2 & 0x03;
        ++i;

        uint8_t *sensorData = packet->data + i;
        unsigned int sensorDataBytes = packet->data_length - i;

        if(f < fingers.size())
        {
            i += fingers[f]->setNewSensorValue(sensorType, sensorData, sensorDataBytes, &errorFlag);
        }

        if(errorFlag)
            break;
    }
}