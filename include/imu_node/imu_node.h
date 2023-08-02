/*
  Copyright 2023 Alapaca-zip

  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
*/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <serial/serial.h>

#include <numeric>

class ImuNode
{
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  ros::Publisher _imu_pub;
  std::string _port;
  std::string _frame_id;
  std::string _imu_topic;
  std::vector<uint8_t> _buff;
  std::vector<float> _acceleration;
  std::vector<float> _angular_velocity;
  std::vector<float> _quaternion;
  sensor_msgs::Imu _imu_msg;
  serial::Serial _imu_serial;
  bool _acceleration_flag;
  bool _angular_velocity_flag;
  bool _quaternion_flag;
  int _baudrate;
  float _time_out;

  const size_t _BUFFER_SIZE = 11;
  const uint8_t _HEADER_BYTE = 0x55;
  const uint8_t _ACCELERATION_BYTE = 0x51;
  const uint8_t _ANGULAR_VELOCITY_BYTE = 0x52;
  const uint8_t _QUATERNION_BYTE = 0x53;

public:
  ImuNode();
  void controlLoop();
  void openSerial(const std::string & port, const int & baudrate, const float & time_out);
  void publishMsg(
    const std::vector<float> & acceleration, const std::vector<float> & angular_velocity,
    const std::vector<float> & quaternion);
  void processData(const uint8_t & raw_data);
  bool checkSum(
    const std::vector<uint8_t>::iterator & begin, const std::vector<uint8_t>::iterator & end,
    const uint8_t & check_data);
  std::vector<int16_t> hexToShort(const std::vector<uint8_t> & raw_data);
  std::vector<float> processAccelerationData(const std::vector<int16_t> & data);
  std::vector<float> processAngularVelocityData(const std::vector<int16_t> & data);
  std::vector<float> processQuaternionData(const std::vector<int16_t> & data);
};
