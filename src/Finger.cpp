#include "Finger.h"
#include <iostream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

// Static member initialisation (unchanged)
int         Finger::fingerCount              = 0;
const float Finger::BETA                    = 0.1f;
const float Finger::SAMPLE_FREQ             = 1000.0f;
const int   Finger::BIAS_CALCULATION_ITERATIONS = 5000;
const float Finger::ACCEL_RES              = 2.0f / 32768.0f;
const float Finger::GYRO_RES               = 250.0f / 32768.0f;

/****************************************************************************************
//  Finger::Finger()
//  ROS 2: receives the shared node pointer to create publishers on the correct node.
****************************************************************************************/
Finger::Finger(rclcpp::Node::SharedPtr node) :
    node_(node),
    temperature(0),
    initDone(false),
    biasCalculationIteration(0),
    norm_bias(0),
    stopThread(false)
{
    sensorId = ++fingerCount;
    q.value = {1.0f, 0.0f, 0.0f, 0.0f};   // init quaternion (adjust to match your .msg array field)

    // Create all publishers up front in the constructor (thread-safe in ROS 2)
    std::string prefix = "TactileSensor4/Sensor" + std::to_string(sensorId) + "/";

    staticData_pub_    = node_->create_publisher<tactilesensors4::msg::StaticData>   (prefix + "StaticData",    1000);
    dynamic_pub_       = node_->create_publisher<tactilesensors4::msg::Dynamic>      (prefix + "Dynamic",       1000);
    accelerometer_pub_ = node_->create_publisher<tactilesensors4::msg::Accelerometer>(prefix + "Accelerometer", 1000);
    eulerAngle_pub_    = node_->create_publisher<tactilesensors4::msg::EulerAngle>   (prefix + "EulerAngle",    1000);
    gyroscope_pub_     = node_->create_publisher<tactilesensors4::msg::Gyroscope>    (prefix + "Gyroscope",     1000);
    magnetometer_pub_  = node_->create_publisher<tactilesensors4::msg::Magnetometer> (prefix + "Magnetometer",  1000);
    quaternion_pub_    = node_->create_publisher<tactilesensors4::msg::Quaternion>   (prefix + "Quaternion",    1000);

    publisherThread = std::thread(&Finger::runPublisher, this);
}

/****************************************************************************************
//  Finger::~Finger()
****************************************************************************************/
Finger::~Finger()
{
    stopThread = true;
    if (publisherThread.joinable())
    {
        completeDataCondition.notify_all();
        publisherThread.join();
    }
}

/****************************************************************************************
//  Finger::runPublisher()
//  ROS 2 version: no NodeHandle, no ros::ok(), no loop_rate.sleep() from ROS.
//  Uses rclcpp::ok() and std::chrono for rate control.
****************************************************************************************/
void Finger::runPublisher()
{
    // 1000 Hz → 1 ms period
    auto period = std::chrono::microseconds(1000);

    while (rclcpp::ok() && !stopThread)
    {
        std::unique_lock<std::mutex> lock(completeDataMutex);

        auto status = completeDataCondition.wait_until(
            lock,
            std::chrono::system_clock::now() + 500ms
        );

        if (status == std::cv_status::timeout)
        {
            std::cerr << "No data received for Sensor" << sensorId << "..." << std::endl;
            continue;
        }

        if (stopThread) break;

        {
            std::unique_lock<std::mutex> publishLock(publishingMutex);
            updateIMU();

            // Publish all topics (same as ROS 1, just ->publish() instead of .publish())
            dynamic_pub_->publish(dynamic);
            staticData_pub_->publish(staticData);
            accelerometer_pub_->publish(accelerometer);
            gyroscope_pub_->publish(gyroscope);
            magnetometer_pub_->publish(magnetometer);

            if (initDone)
            {
                eulerAngle_pub_->publish(eulerAngle);
                quaternion_pub_->publish(q);
            }
        }

        // Maintain ~1000 Hz publish rate without ros::Rate
        std::this_thread::sleep_for(period);
    }
}

/****************************************************************************************
//  Finger::updateIMU()  — unchanged logic, only variable names adapted to msg fields
****************************************************************************************/
void Finger::updateIMU()
{
    if (initDone)
    {
        float ax = accelerometer.value[0] * ACCEL_RES - accelBias[0];
        float ay = accelerometer.value[1] * ACCEL_RES - accelBias[1];
        float az = accelerometer.value[2] * ACCEL_RES - accelBias[2];

        float gx = gyroscope.value[0] * GYRO_RES - gyroBias[0];
        float gy = gyroscope.value[1] * GYRO_RES - gyroBias[1];
        float gz = gyroscope.value[2] * GYRO_RES - gyroBias[2];

        madgwickAHRSUpdateIMU(
            gx * M_PI / 180.0f,
            gy * M_PI / 180.0f,
            gz * M_PI / 180.0f,
            ax, ay, az
        );

        eulerAngle.value[0] = atan2f(
            2.0f*(q.value[0]*q.value[1] + q.value[2]*q.value[3]),
            q.value[0]*q.value[0] - q.value[1]*q.value[1] - q.value[2]*q.value[2] + q.value[3]*q.value[3]
        ) * 180.0f / M_PI;

        eulerAngle.value[1] = -asinf(
            2.0f*(q.value[1]*q.value[3] - q.value[0]*q.value[2])
        ) * 180.0f / M_PI;

        eulerAngle.value[2] = atan2f(
            2.0f*(q.value[1]*q.value[2] + q.value[0]*q.value[3]),
            q.value[0]*q.value[0] + q.value[1]*q.value[1] - q.value[2]*q.value[2] - q.value[3]*q.value[3]
        ) * 180.0f / M_PI;
    }
    else
    {
        initBias();
    }
}

/****************************************************************************************
//  Finger::initBias()  — 100% unchanged logic
****************************************************************************************/
void Finger::initBias()
{
    if (biasCalculationIteration < BIAS_CALCULATION_ITERATIONS)
    {
        gyroBias[0]  += gyro[0]   * GYRO_RES;
        gyroBias[1]  += gyro[1]   * GYRO_RES;
        gyroBias[2]  += gyro[2]   * GYRO_RES;
        accelBias[0] += accel[0]  * ACCEL_RES;
        accelBias[1] += accel[1]  * ACCEL_RES;
        accelBias[2] += accel[2]  * ACCEL_RES;
        biasCalculationIteration++;
    }
    else
    {
        gyroBias[0]  /= biasCalculationIteration;
        gyroBias[1]  /= biasCalculationIteration;
        gyroBias[2]  /= biasCalculationIteration;
        accelBias[0] /= biasCalculationIteration;
        accelBias[1] /= biasCalculationIteration;
        accelBias[2] /= biasCalculationIteration;

        norm_bias = sqrtf(
            accelBias[0]*accelBias[0] +
            accelBias[1]*accelBias[1] +
            accelBias[2]*accelBias[2]
        ) - 1.0f;

        float sum = accelBias[0] + accelBias[1] + accelBias[2];
        accelBias[0] *= norm_bias / sum;
        accelBias[1] *= norm_bias / sum;
        accelBias[2] *= norm_bias / sum;

        initDone = true;
    }
}

/****************************************************************************************
//  Finger::setNewSensorValue()  — 100% unchanged logic
****************************************************************************************/
int Finger::setNewSensorValue(int sensorType, uint8_t *data, unsigned int size, bool* errorFlag)
{
    int byteRead;
    std::unique_lock<std::mutex> publishLock(publishingMutex);

    switch (sensorType)
    {
        case USB_SENSOR_TYPE_DYNAMIC_TACTILE:
            byteRead = extractUint16((uint16_t *)dynamicTactile, FINGER_DYNAMIC_TACTILE_COUNT, data, size);
            dynamic.value = dynamicTactile[0];
            completeDataCondition.notify_all();
            break;

        case USB_SENSOR_TYPE_STATIC_TACTILE:
            byteRead = extractUint16(staticTactile, FINGER_STATIC_TACTILE_COUNT, data, size);
            std::copy(std::begin(staticTactile), std::end(staticTactile), std::begin(staticData.value));
            break;

        case USB_SENSOR_TYPE_ACCELEROMETER:
            byteRead = extractUint16((uint16_t *)accel, 3, data, size);
            std::copy(std::begin(accel), std::end(accel), std::begin(accelerometer.value));
            break;

        case USB_SENSOR_TYPE_GYROSCOPE:
            byteRead = extractUint16((uint16_t *)gyro, 3, data, size);
            std::copy(std::begin(gyro), std::end(gyro), std::begin(gyroscope.value));
            break;

        case USB_SENSOR_TYPE_MAGNETOMETER:
            byteRead = extractUint16((uint16_t *)magnet, 3, data, size);
            std::copy(std::begin(magnet), std::end(magnet), std::begin(magnetometer.value));
            break;

        case USB_SENSOR_TYPE_TEMPERATURE:
            byteRead = extractUint16((uint16_t *)&temperature, 1, data, size);
            break;

        default:
            std::cerr << "Unrecognised sensor type -> " << sensorType << std::endl;
            *errorFlag = true;
            byteRead = -1;
    }
    return byteRead;
}

/****************************************************************************************
//  Madgwick AHRS — 100% unchanged
****************************************************************************************/
void Finger::madgwickAHRSUpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q.value[1]*gx - q.value[2]*gy - q.value[3]*gz);
    qDot2 = 0.5f * ( q.value[0]*gx + q.value[2]*gz - q.value[3]*gy);
    qDot3 = 0.5f * ( q.value[0]*gy - q.value[1]*gz + q.value[3]*gx);
    qDot4 = 0.5f * ( q.value[0]*gz + q.value[1]*gy - q.value[2]*gx);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        _2q0 = 2.0f*q.value[0]; _2q1 = 2.0f*q.value[1];
        _2q2 = 2.0f*q.value[2]; _2q3 = 2.0f*q.value[3];
        _4q0 = 4.0f*q.value[0]; _4q1 = 4.0f*q.value[1]; _4q2 = 4.0f*q.value[2];
        _8q1 = 8.0f*q.value[1]; _8q2 = 8.0f*q.value[2];
        q0q0 = q.value[0]*q.value[0]; q1q1 = q.value[1]*q.value[1];
        q2q2 = q.value[2]*q.value[2]; q3q3 = q.value[3]*q.value[3];

        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
        s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q.value[1] - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
        s2 = 4.0f*q0q0*q.value[2] + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
        s3 = 4.0f*q1q1*q.value[3] - _2q1*ax + 4.0f*q2q2*q.value[3] - _2q2*ay;

        recipNorm = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        qDot1 -= BETA*s0; qDot2 -= BETA*s1; qDot3 -= BETA*s2; qDot4 -= BETA*s3;
    }

    q.value[0] += qDot1 * (1.0f / SAMPLE_FREQ);
    q.value[1] += qDot2 * (1.0f / SAMPLE_FREQ);
    q.value[2] += qDot3 * (1.0f / SAMPLE_FREQ);
    q.value[3] += qDot4 * (1.0f / SAMPLE_FREQ);

    recipNorm = invSqrt(
        q.value[0]*q.value[0] + q.value[1]*q.value[1] +
        q.value[2]*q.value[2] + q.value[3]*q.value[3]
    );
    q.value[0] *= recipNorm; q.value[1] *= recipNorm;
    q.value[2] *= recipNorm; q.value[3] *= recipNorm;
}

float Finger::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

inline uint16_t Finger::parseBigEndian2(const uint8_t *data)
{
    return (uint16_t)data[0] << 8 | data[1];
}

uint8_t Finger::extractUint16(uint16_t *to, uint16_t toCount, uint8_t *data, unsigned int size)
{
    unsigned int cur;
    for (cur = 0; 2*cur + 1 < size && cur < toCount; ++cur)
        to[cur] = parseBigEndian2(&data[2*cur]);
    return cur * 2;
}