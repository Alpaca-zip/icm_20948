#!/usr/bin/env python
import rclpy
import struct
import math
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNode(Node):
    def __init__(self):
        super().__init__("imu_node")
        # Initialize parameters with default values
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("time_out", 0.5)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("imu_topic", "imu")
        self.declare_parameter("frame_id", "imu_link")
        port = self.get_parameter("port").get_parameter_value().string_value
        time_out = self.get_parameter("time_out").get_parameter_value().double_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        imu_topic = self.get_parameter("imu_topic").get_parameter_value().string_value
        self.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )
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
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)

    def open_serial(self, port, baudrate, time_out):
        self.get_logger().info("Port:%s baud:%d" % (port, baudrate))
        try:
            # Attempt to open the serial port
            self.imu_serial = serial.Serial(
                port=port, baudrate=baudrate, timeout=time_out
            )
            self.get_logger().info("\033[32mSerial port opened successfully!\033[0m")
        except Exception as e:
            print(e)
            self.get_logger().error("Serial port opening failure")
            exit(0)

    def control_loop(self):
        try:
            # Check if there is any data waiting to be read from the serial port
            buff_count = self.imu_serial.inWaiting()
        except Exception as e:
            print(e)
            self.get_logger().error("Imu disconnect")
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
                    self.get_logger().warn("0x51 Check failure")

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
                    self.get_logger().warn("0x52 Check failure")

            elif self.buff[1] == 0x53:
                if self.check_sum(data_buff[0:10], data_buff[10]):
                    self.quaternion = [
                        self.hex_to_short(data_buff[2:10])[i] / 32768.0
                        for i in range(0, 4)
                    ]
                    pub_flag = True
                else:
                    self.get_logger().warn("0x53 Check failure")

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
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
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


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ImuNode()
        while rclpy.ok():
            node.control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
