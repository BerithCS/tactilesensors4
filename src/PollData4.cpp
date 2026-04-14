/******************************************************************************************
// PollData V4.0 - Ported to ROS 2 (rclcpp)
// Original Author: Jean-Philippe Roberge Ing, M.Sc.A.
// ROS2 Port: 2026
// Affiliations: Laboratoire de commande et de robotique (ÉTS)
******************************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "tactilesensors4/srv/tactile_sensors.hpp"   // ROS2 uses snake_case for generated headers
#include "Communication.h"
#include <algorithm>
#include <unistd.h>
#include <memory>
#include <vector>
#include <string>

#define DEVICE_OPTION "-device"

using namespace std;

// Forward declarations
bool cmdOptionExists(char** begin, char** end, const string& option);
std::vector<std::string> getDevicesOption(char** begin, char** end, const string& option);

// Global comm list (Communication objects publish internally via the node)
std::vector<std::unique_ptr<Communication>> commList;

// -----------------------------------------------
// ROS 2 Node class
// -----------------------------------------------
class PollDataNode : public rclcpp::Node
{
public:
    PollDataNode() : Node("PollData")
    {
        // Create service (replaces ros::ServiceServer)
        service_ = this->create_service<tactilesensors4::srv::TactileSensors>(
            "Tactile_Sensors_Service",
            std::bind(&PollDataNode::serviceCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Tactile Sensors Service is ready.");
    }

private:
    rclcpp::Service<tactilesensors4::srv::TactileSensors>::SharedPtr service_;

    void serviceCallback(
        const std::shared_ptr<tactilesensors4::srv::TactileSensors::Request> req,
        std::shared_ptr<tactilesensors4::srv::TactileSensors::Response> res)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Tactile Sensors Service received request: [%s]",
                    req->request.c_str());

        if (req->request == "start" || req->request == "Start")
        {
            for (auto& comm : commList)
                comm->enableComm(true);
            res->response = true;
        }
        else if (req->request == "stop" || req->request == "Stop")
        {
            for (auto& comm : commList)
                comm->enableComm(false);
            res->response = true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                        "Tactile Sensor Service received an invalid request.");
            res->response = false;
        }
    }
};

// -----------------------------------------------
// Main
// -----------------------------------------------
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PollDataNode>();

    // Parse device arguments (same logic as ROS 1 version)
    std::vector<std::string> devices;

    if (cmdOptionExists(argv, argv + argc, DEVICE_OPTION))
    {
        devices = getDevicesOption(argv, argv + argc, DEVICE_OPTION);
        if (devices.empty())
        {
            RCLCPP_ERROR(node->get_logger(), "No devices specified after -device flag!");
            return 1;
        }
    }
    else
    {
        // Default device
        devices.emplace_back("/dev/ttyACM0");
    }

    // Create a Communication object per device
    // NOTE: Communication internally needs the rclcpp node to create publishers.
    //       Pass the node shared_ptr into Communication's constructor.
    //       You will need to update Communication.h/.cpp accordingly (see note below).
    for (auto& name : devices)
    {
        commList.emplace_back(std::make_unique<Communication>(node, &name));
    }

    RCLCPP_INFO(node->get_logger(), "PollData4 running. Spinning...");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// -----------------------------------------------
// Helper functions (identical to ROS 1 version)
// -----------------------------------------------
bool cmdOptionExists(char** begin, char** end, const string& option)
{
    return find(begin, end, option) != end;
}

std::vector<std::string> getDevicesOption(char** begin, char** end, const string& option)
{
    std::vector<std::string> devices;
    char** itr = find(begin, end, option);
    while (itr != end && ++itr != end)
        devices.emplace_back(*itr);
    return devices;
}