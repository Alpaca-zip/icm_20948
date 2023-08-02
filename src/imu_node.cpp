/*
  Copyright 2023 Alapaca-zip

  This software is released under the MIT License.
  http://opensource.org/licenses/mit-license.php
*/

#include "imu_node/imu_node.h"

ImuNode::ImuNode() : _pnh("~")
{
  _pnh.param<std::string>("port", _port, "/dev/ttyACM0");
  _pnh.param<std::string>("frame_id", _frame_id, "imu_link");
  _pnh.param<std::string>("imu_topic", _imu_topic, "imu");
  _pnh.param<float>("time_out", _time_out, 0.5);
  _pnh.param<int>("baudrate", _baudrate, 115200);
  _imu_pub = _nh.advertise<sensor_msgs::Imu>(_imu_topic, 10);
  _acceleration_flag = false;
  _angular_velocity_flag = false;
  _quaternion_flag = false;
  openSerial(_port, _baudrate, _time_out);
}

void ImuNode::openSerial(const std::string & port, const int & baudrate, const float & time_out)
{
  ROS_INFO("Port:%s baud:%d", port.c_str(), baudrate);
  serial::Timeout to = serial::Timeout::simpleTimeout(time_out);

  while (ros::ok()) {
    try {
      // Attempt to open the serial port
      _imu_serial.setPort(port);
      _imu_serial.setBaudrate(baudrate);
      _imu_serial.setTimeout(to);
      _imu_serial.open();
      ROS_INFO("\033[32mSerial port opened successfully!\033[0m");
      break;
    } catch (serial::IOException & e) {
      ROS_WARN("Serial port opening failure...Retrying");
      ros::Duration(5.0).sleep();
    }
  }
}

void ImuNode::controlLoop()
{
  try {
    // Check if there is any data waiting to be read from the serial port
    if (_imu_serial.available()) {
      // Read the waiting data and process it
      std::vector<uint8_t> buff_data;
      _imu_serial.read(buff_data, _imu_serial.available());
      for (const auto & byte : buff_data) {
        processData(byte);
      }
    }
  } catch (const std::exception & e) {
    ROS_WARN("IMU disconnect...Retrying");
    openSerial(_port, _baudrate, _time_out);
  }
}

void ImuNode::processData(const uint8_t & raw_data)
{
  // Process each byte of raw data from the serial port
  _buff.push_back(raw_data);

  if (_buff[0] != _HEADER_BYTE) {
    _buff.clear();
    return;
  }

  if (_buff.size() < _BUFFER_SIZE) {
    return;
  }

  if (!checkSum(_buff.begin(), _buff.begin() + _BUFFER_SIZE - 1, _buff[_BUFFER_SIZE - 1])) {
    ROS_WARN("Check failure for byte: %d", _buff[1]);
    _buff.clear();
    return;
  }

  // Compute common variables
  std::vector<uint8_t> sliced_data(_buff.begin() + 2, _buff.begin() + 10);
  std::vector<int16_t> data = hexToShort(sliced_data);

  if (_buff[1] == _ACCELERATION_BYTE) {
    // Interpret acceleration data
    _acceleration = processAccelerationData(data);
    _acceleration_flag = true;
  } else if (_buff[1] == _ANGULAR_VELOCITY_BYTE) {
    // Interpret angular velocity data
    _angular_velocity = processAngularVelocityData(data);
    _angular_velocity_flag = true;
  } else if (_buff[1] == _QUATERNION_BYTE) {
    // Interpret quaternion data
    _quaternion = processQuaternionData(data);
    _quaternion_flag = true;
  }

  if (_acceleration_flag && _angular_velocity_flag && _quaternion_flag) {
    publishMsg(_acceleration, _angular_velocity, _quaternion);
    _acceleration_flag = false;
    _angular_velocity_flag = false;
    _quaternion_flag = false;
  }

  _buff.clear();
  return;
}

std::vector<int16_t> ImuNode::hexToShort(const std::vector<uint8_t> & raw_data)
{
  // Convert a list of bytes to a list of shorts
  std::vector<int16_t> result;
  for (std::vector<uint8_t>::const_iterator it = raw_data.begin(); it != raw_data.end(); it += 2) {
    int16_t value = (static_cast<int16_t>(*(it + 1)) << 8) | *it;
    result.push_back(value);
  }
  return result;
}

bool ImuNode::checkSum(
  const std::vector<uint8_t>::iterator & begin, const std::vector<uint8_t>::iterator & end,
  const uint8_t & check_data)
{
  // Check the checksum of the data
  uint8_t sum = std::accumulate(begin, end, 0);
  return (sum & 0xFF) == check_data;
}

std::vector<float> ImuNode::processAccelerationData(const std::vector<int16_t> & data)
{
  std::vector<float> acceleration(3);
  for (size_t i = 0; i < 3; i++) {
    acceleration[i] = data[i] / 32768.0f * 4 * 9.80665;
  }
  return acceleration;
}

std::vector<float> ImuNode::processAngularVelocityData(const std::vector<int16_t> & data)
{
  std::vector<float> angular_velocity(3);
  for (size_t i = 0; i < 3; i++) {
    angular_velocity[i] = data[i] / 32768.0f * 2000 * M_PI / 180;
  }
  return angular_velocity;
}

std::vector<float> ImuNode::processQuaternionData(const std::vector<int16_t> & data)
{
  std::vector<float> quaternion(4);
  for (size_t i = 0; i < 4; i++) {
    quaternion[i] = data[i] / 32768.0f;
  }
  return quaternion;
}

void ImuNode::publishMsg(
  const std::vector<float> & acceleration, const std::vector<float> & angular_velocity,
  const std::vector<float> & quaternion)
{
  // Construct and publish the IMU message
  _imu_msg.header.stamp = ros::Time::now();
  _imu_msg.header.frame_id = _frame_id;
  _imu_msg.linear_acceleration.x = acceleration[0];
  _imu_msg.linear_acceleration.y = acceleration[1];
  _imu_msg.linear_acceleration.z = acceleration[2];
  _imu_msg.angular_velocity.x = angular_velocity[0];
  _imu_msg.angular_velocity.y = angular_velocity[1];
  _imu_msg.angular_velocity.z = angular_velocity[2];
  _imu_msg.orientation.x = quaternion[0];
  _imu_msg.orientation.y = quaternion[1];
  _imu_msg.orientation.z = quaternion[2];
  _imu_msg.orientation.w = quaternion[3];

  _imu_pub.publish(_imu_msg);
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "imu_node");
  ImuNode node;
  while (ros::ok()) {
    node.controlLoop();
  }
  return 0;
}
