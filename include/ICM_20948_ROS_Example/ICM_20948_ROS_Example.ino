#define USE_USBCON // seeeduino xiao model needs this flag to be defined

//Includes ROS Serial Arduino library and SparkFun ICM-20948 Arduino Library
//More info...
//ROS Serial Arduino library : https://github.com/frankjoshua/rosserial_arduino_lib
//SparkFun ICM-20948 Arduino Library : https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "ICM_20948.h" 

#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.
#define AD0_VAL 1      // The value of the last bit of the I2C address.                \
                       // On the SparkFun 9DoF IMU breakout the default is 1, and when \
                       // the ADR jumper is closed the value becomes 0
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

ros::NodeHandle nh;

geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

void setup(){
  SERIAL_PORT.begin(115200); // Start the serial console
  nh.getHardware()->setBaud(115200);
  nh.initNode();nh.getHardware()->setBaud(115200);
  nh.advertise(imu_pub);
  tfbroadcaster.init(nh);

  delay(100);

  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();
    
  delay(1000);

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized){

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

    myICM.begin(WIRE_PORT, AD0_VAL);

    if (myICM.status != ICM_20948_Stat_Ok){
      
      delay(500);
      
    }else{
      initialized = true;
    }
  }

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  
  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0.375) == ICM_20948_Stat_Ok); // Set to 40Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0.375) == ICM_20948_Stat_Ok); // Set to 40Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0.375) == ICM_20948_Stat_Ok); // Set to 40Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0.375) == ICM_20948_Stat_Ok); // Set to 40Hz
  
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
}

void loop(){
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  imu_msg.header.stamp    = nh.now();
  imu_msg.header.frame_id = "imu_link";

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)){ // Was valid data available?
    if ((data.header & DMP_header_bitmap_Quat6) > 0){ // We have asked for GRV data so we should receive Quat6
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.
      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = - ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      imu_msg.orientation.w = q0;
      imu_msg.orientation.x = q1;
      imu_msg.orientation.y = q2;
      imu_msg.orientation.z = q3;

      tfs_msg.header.stamp    = nh.now();
      tfs_msg.header.frame_id = "base_link";
      tfs_msg.child_frame_id  = "imu_link";
      tfs_msg.transform.rotation.w = q0;
      tfs_msg.transform.rotation.x = q1;
      tfs_msg.transform.rotation.y = q2;
      tfs_msg.transform.rotation.z = q3;

      tfs_msg.transform.translation.x = 0.0;
      tfs_msg.transform.translation.y = 0.0;
      tfs_msg.transform.translation.z = 0.0;

      tfbroadcaster.sendTransform(tfs_msg);
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0){ // Check for Accel
      float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y;
      float acc_z = (float)data.Raw_Accel.Data.Z;

      imu_msg.linear_acceleration.x = acc_x;
      imu_msg.linear_acceleration.y = acc_y;
      imu_msg.linear_acceleration.z = acc_z;
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0){ // Check for Gyro
      float x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      float y = (float)data.Raw_Gyro.Data.Y;
      float z = (float)data.Raw_Gyro.Data.Z;

      imu_msg.angular_velocity.x = x;
      imu_msg.angular_velocity.y = y;
      imu_msg.angular_velocity.z = z;
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail){ // If more data is available then we should read it right away - and not delay
    
    delay(10);
    
  }
  
  imu_pub.publish(&imu_msg);
  
  nh.spinOnce();
}
