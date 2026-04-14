#ifndef TACTILESENSORS4_COMMUNICATION_H
#define TACTILESENSORS4_COMMUNICATION_H

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <thread>
#include <vector>
#include "Finger.h"

#define READ_DATA_PERIOD_MS 1

enum UsbPacketSpecial
{
    USB_PACKET_START_BYTE = 0x9A
};

enum UsbCommands
{
    USB_COMMAND_READ_SENSORS = 0x61,
    USB_COMMAND_AUTOSEND_SENSORS = 0x58,
    USB_COMMAND_ENTER_BOOTLOADER = 0xE2
};

struct UsbPacket
{
    uint8_t start_byte;
    uint8_t crc8;           // over command, data_length and data
    uint8_t command;        // 4 bits of flag (MSB) and 4 bits of command (LSB)
    uint8_t data_length;
    uint8_t data[60];
};


class Communication{
public:
    explicit Communication(rclcpp::Node::SharedPtr node, const std::string* deviceName);

    virtual ~Communication();

    void enableComm(bool enable);

private:
    void run();
    static bool OpenAndConfigurePort(int *USB, char const *TheDevice);
    void usbSend(UsbPacket *packet) const;
    static uint8_t calcCrc8(uint8_t *data, size_t len);
    static bool usbReadByte(UsbPacket *packet, unsigned int *readSoFar, uint8_t d);
    void parseSensors(UsbPacket *packet);

    std::vector<std::unique_ptr<Finger>> fingers;

    int USB;

    std::thread comThread;
    std::condition_variable enableCondition;
    std::mutex enableMutex;
    bool stopThread;
    bool isEnabled;

};


#endif //TACTILESENSORS4_COMMUNICATION_H
