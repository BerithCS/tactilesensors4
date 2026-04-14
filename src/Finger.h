#ifndef TACTILESENSORS4_FINGER_H
#define TACTILESENSORS4_FINGER_H

#include <cstdint>
#include <thread>
#include <condition_variable>
#include <mutex>

// ROS 2 includes (replace all ros/ros.h and tactilesensors4 ROS1 msg headers)
#include "rclcpp/rclcpp.hpp"
#include "tactilesensors4/msg/quaternion.hpp"
#include "tactilesensors4/msg/dynamic.hpp"
#include "tactilesensors4/msg/accelerometer.hpp"
#include "tactilesensors4/msg/euler_angle.hpp"
#include "tactilesensors4/msg/magnetometer.hpp"
#include "tactilesensors4/msg/static_data.hpp"
#include "tactilesensors4/msg/gyroscope.hpp"

#define FINGER_STATIC_TACTILE_ROW 4
#define FINGER_STATIC_TACTILE_COL 7
#define FINGER_STATIC_TACTILE_COUNT (FINGER_STATIC_TACTILE_ROW * FINGER_STATIC_TACTILE_COL)
#define FINGER_DYNAMIC_TACTILE_COUNT 1

enum UsbSensorType
{
    USB_SENSOR_TYPE_STATIC_TACTILE  = 0x10,
    USB_SENSOR_TYPE_DYNAMIC_TACTILE = 0x20,
    USB_SENSOR_TYPE_ACCELEROMETER   = 0x30,
    USB_SENSOR_TYPE_GYROSCOPE       = 0x40,
    USB_SENSOR_TYPE_MAGNETOMETER    = 0x50,
    USB_SENSOR_TYPE_TEMPERATURE     = 0x60
};

class Finger {
public:
    // ROS 2: constructor now receives the node so it can create publishers
    explicit Finger(rclcpp::Node::SharedPtr node);
    virtual ~Finger();

    int setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag);

private:
    void runPublisher();
    void updateIMU();
    void initBias();
    void madgwickAHRSUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    static uint8_t extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size);
    static inline uint16_t parseBigEndian2(const uint8_t *data);
    static float invSqrt(float x);

    // ROS 2 node reference (shared with Communication/PollData4)
    rclcpp::Node::SharedPtr node_;

    // ROS 2 publishers (replaces ros::Publisher)
    rclcpp::Publisher<tactilesensors4::msg::StaticData>::SharedPtr    staticData_pub_;
    rclcpp::Publisher<tactilesensors4::msg::Dynamic>::SharedPtr       dynamic_pub_;
    rclcpp::Publisher<tactilesensors4::msg::Accelerometer>::SharedPtr accelerometer_pub_;
    rclcpp::Publisher<tactilesensors4::msg::EulerAngle>::SharedPtr    eulerAngle_pub_;
    rclcpp::Publisher<tactilesensors4::msg::Gyroscope>::SharedPtr     gyroscope_pub_;
    rclcpp::Publisher<tactilesensors4::msg::Magnetometer>::SharedPtr  magnetometer_pub_;
    rclcpp::Publisher<tactilesensors4::msg::Quaternion>::SharedPtr    quaternion_pub_;

    // Sensor data buffers (unchanged from ROS 1)
    uint16_t staticTactile[FINGER_STATIC_TACTILE_COUNT] = {};
    int16_t  dynamicTactile[FINGER_DYNAMIC_TACTILE_COUNT] = {};
    int16_t  accel[3] = {};
    float    accelBias[3] = {};
    int16_t  gyro[3] = {};
    float    gyroBias[3] = {};
    int16_t  magnet[3] = {};
    int16_t  temperature;

    bool  initDone;
    int   biasCalculationIteration;
    float norm_bias;
    int   sensorId;

    // ROS 2 message instances (snake_case namespaced)
    tactilesensors4::msg::Quaternion    q;
    tactilesensors4::msg::Dynamic       dynamic;
    tactilesensors4::msg::Accelerometer accelerometer;
    tactilesensors4::msg::EulerAngle    eulerAngle;
    tactilesensors4::msg::Magnetometer  magnetometer;
    tactilesensors4::msg::StaticData    staticData;
    tactilesensors4::msg::Gyroscope     gyroscope;

    std::thread            publisherThread;
    std::condition_variable completeDataCondition;
    std::mutex             completeDataMutex;
    std::mutex             publishingMutex;
    bool                   stopThread;

    static const float BETA;
    static const float SAMPLE_FREQ;
    static const int   BIAS_CALCULATION_ITERATIONS;
    static const float ACCEL_RES;
    static const float GYRO_RES;
    static int         fingerCount;
};

#endif //TACTILESENSORS4_FINGER_H
