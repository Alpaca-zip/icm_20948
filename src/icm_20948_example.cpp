#include "icm_20948_example.h"

/* ++++++++++++++++++++++++++++++++++
       ICM_20948_EXAMPLE class
++++++++++++++++++++++++++++++++++ */
ICM_20948_EXAMPLE::ICM_20948_EXAMPLE(){
  init();
}

void ICM_20948_EXAMPLE::init(){
  imu_sub = nh.subscribe("imu", 10, &ICM_20948_EXAMPLE::monitor_imu_callback, this);
}

void ICM_20948_EXAMPLE::monitor_imu_callback(const sensor_msgs::Imu& imu_msg){
  //ROS_INFO("quat_w: %f, quat_x: %f, quat_y: %f, quat_z: %f", imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z);
  //ROS_INFO("acc_x: %f, acc_y: %f, acc_z: %f", imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
  //ROS_INFO("gyro_x: %f, gyro_y: %f, gyro_z: %f", imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);

  q2sqr = imu_msg.orientation.y * imu_msg.orientation.y;

  // roll (x-axis rotation)
  t0 = 2.0 * (imu_msg.orientation.w * imu_msg.orientation.x + imu_msg.orientation.y * imu_msg.orientation.z);
  t1 = 1.0 - 2.0 * (imu_msg.orientation.x * imu_msg.orientation.x + q2sqr);
  roll = atan2(t0, t1) * 180.0 / M_PI;
  // pitch (y-axis rotation)
  t2 = 2.0 * (imu_msg.orientation.w * imu_msg.orientation.y - imu_msg.orientation.z * imu_msg.orientation.x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  pitch = asin(t2) * 180.0 / M_PI;
  // yaw (z-axis rotation)
  t3 = 2.0 * (imu_msg.orientation.w * imu_msg.orientation.z + imu_msg.orientation.x * imu_msg.orientation.y);
  t4 = 1.0 - 2.0 * (q2sqr + imu_msg.orientation.z * imu_msg.orientation.z);
  yaw = atan2(t3, t4) * 180.0 / M_PI;

  ROS_INFO("roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);
}

/* ++++++++++++++++++++++++++++++++++
               main
++++++++++++++++++++++++++++++++++ */
int main(int argc, char **argv){
  ros::init(argc, argv, "icm_20948_example"); 
  ICM_20948_EXAMPLE ICM_20948;
  ros::spin();
  return 0;
}
