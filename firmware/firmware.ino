#define USE_USBCON // seeeduino xiao model needs this flag to be defined
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1
#define PI 3.1415926535
#define MG 0.00980665

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "ICM_20948.h"

ICM_20948_I2C myICM;

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher mag_pub("mag", &mag_msg);

void setup() {
  SERIAL_PORT.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.initNode(); nh.getHardware()->setBaud(115200);
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  delay(100);

  while (SERIAL_PORT.available())
    SERIAL_PORT.read();
  delay(1000);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status != ICM_20948_Stat_Ok) {
      delay(500);
    } else {
      initialized = true;
    }
  }

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
}

void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  myICM.getAGMT();
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";
  mag_msg.header.stamp = imu_msg.header.stamp;
  mag_msg.header.frame_id = "mag_link";

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0;
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0;
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0;
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      imu_msg.orientation.w = q0;
      imu_msg.orientation.x = q1;
      imu_msg.orientation.y = q2;
      imu_msg.orientation.z = q3;
    }
    imu_msg.linear_acceleration.x = myICM.accX() * MG;
    imu_msg.linear_acceleration.y = myICM.accY() * MG;
    imu_msg.linear_acceleration.z = myICM.accZ() * MG;

    imu_msg.angular_velocity.x = myICM.gyrX() * (PI / 180);
    imu_msg.angular_velocity.y = myICM.gyrY() * (PI / 180);
    imu_msg.angular_velocity.z = myICM.gyrZ() * (PI / 180);


    if ((data.header & DMP_header_bitmap_Compass) > 0) {
      mag_msg.magnetic_field.x = (float)data.Compass.Data.X;
      mag_msg.magnetic_field.y = (float)data.Compass.Data.Y;
      mag_msg.magnetic_field.z = (float)data.Compass.Data.Z;
      mag_pub.publish(&mag_msg);
    }
  }
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
}
