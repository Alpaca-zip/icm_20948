#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class ICM_20948_EXAMPLE{
  public:
  ICM_20948_EXAMPLE();

  private:
  double q2sqr;
  double t0, t1, t2, t3, t4;
  double roll, pitch, yaw;
  ros::NodeHandle nh;
  ros::Subscriber imu_sub;

  void init();
  void monitor_imu_callback(const sensor_msgs::Imu& imu_msg);
};