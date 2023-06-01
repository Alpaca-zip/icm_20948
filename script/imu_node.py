#!/usr/bin/env python
import rospy
import struct
import math
import serial
from sensor_msgs.msg import Imu


class ImuNode:
    def __init__(self):
        # Initialize parameters with default values
        port = rospy.get_param("~port", "/dev/ttyACM0")
        time_out = rospy.get_param("~time_out", 0.5)
        baudrate = rospy.get_param("~baudrate", 115200)
        imu_topic = rospy.get_param("~imu_topic", "imu")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")
        # Initialize other necessary variables
        self.key = 0
        self.buff = {}
        self.angular_velocity = [0, 0, 0]
        self.acceleration = [0, 0, 0]
        self.quaternion = [0, 0, 0]
        # Open the serial connection
        self.open_serial(port, baudrate, time_out)
        # Initialize message and publisher
        self.imu_msg = Imu()
        self.imu_pub = rospy.Publisher(imu_topic, Imu, queue_size=10)

    def open_serial(self, port, baudrate, time_out):
        rospy.loginfo("Port:%s baud:%d" % (port, baudrate))
        try:
            # Attempt to open the serial port
            self.imu_serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=time_out
            )
            rospy.loginfo("\033[32mSerial port opened successfully!\033[0m")
        except Exception as e:
            print(e)
            rospy.logerr("Serial port opening failure")
            exit(0)

    def control_loop(self):
        try:
            # Check if there is any data waiting to be read from the serial port
            buff_count = self.imu_serial.inWaiting()
        except Exception as e:
            print(e)
            rospy.logerr("Imu disconnect")
            exit(0)
        else:
            if buff_count > 0:
                # Read the waiting data and process it
                buff_data = self.imu_serial.read(buff_count)
                for i in range(0, buff_count):
                    if self.process_data(buff_data[i]):
                        self.publish_msg()

    def process_data(self, raw_data):
        # Process each byte of raw data from the serial port
        pub_flag = False
        self.buff[self.key] = raw_data
        self.key += 1

        if self.buff[0] != 0x55:
            self.key = 0
            return

        if self.key < 11:
            return
        else:
            # Process the different types of data based on the second byte
            data_buff = list(self.buff.values())
            if self.buff[1] == 0x51:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.acceleration = [
                        self.hex_to_short(data_buff[2:10])[i] / 32768.0 * 4 * 9.80665
                        for i in range(0, 3)
                    ]
                else:
                    rospy.logwarn("0x51 Check failure")

            elif self.buff[1] == 0x52:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.angular_velocity = [
                        self.hex_to_short(data_buff[2:10])[i]
                        / 32768.0
                        * 2000
                        * math.pi
                        / 180
                        for i in range(0, 3)
                    ]
                else:
                    rospy.logwarn("0x52 Check failure")

            elif self.buff[1] == 0x53:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.quaternion = [
                        self.hex_to_short(data_buff[2:10])[i] / 32768.0
                        for i in range(0, 4)
                    ]
                    pub_flag = True
                else:
                    rospy.logwarn("0x53 Check failure")

            self.buff = {}
            self.key = 0
            return pub_flag

    def check_sum(self, list_data, check_data):
        # Check the checksum of the data
        return sum(list_data) & 0xFF == check_data

    def hex_to_short(self, raw_data):
        # Convert a list of bytes to a list of shorts
        return list(struct.unpack("hhhh", bytearray(raw_data)))

    def publish_msg(self):
        # Construct and publish the IMU message
        self.imu_msg.header.stamp = rospy.get_rostime()
        self.imu_msg.header.frame_id = self.frame_id

        self.imu_msg.orientation.x = self.quaternion[0]
        self.imu_msg.orientation.y = self.quaternion[1]
        self.imu_msg.orientation.z = self.quaternion[2]
        self.imu_msg.orientation.w = self.quaternion[3]

        self.imu_msg.angular_velocity.x = self.angular_velocity[0]
        self.imu_msg.angular_velocity.y = self.angular_velocity[1]
        self.imu_msg.angular_velocity.z = self.angular_velocity[2]
        self.imu_msg.linear_acceleration.x = self.acceleration[0]
        self.imu_msg.linear_acceleration.y = self.acceleration[1]
        self.imu_msg.linear_acceleration.z = self.acceleration[2]
        self.imu_pub.publish(self.imu_msg)


if __name__ == "__main__":
    rospy.init_node("imu_node")
    node = ImuNode()
    while not rospy.is_shutdown():
        node.control_loop()
