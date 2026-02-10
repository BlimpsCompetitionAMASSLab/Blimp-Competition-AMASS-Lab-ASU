# -*- coding: utf-8 -*
import serial
import time
import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import LidarData

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.lidar_data = self.create_publisher(LidarData,"lidar_data",10)
        self.ser = serial.Serial("/dev/ttyS0", 115200, timeout = 1)
        self.create_timer(0.1,self.publish_lidar_data)

    def publish_lidar_data(self):
        msg = LidarData()
        self.counter = self.ser.in_waiting
        if self.counter > 8:
            self.bytes_serial = self.ser.read(9)
            self.ser.reset_input_buffer()
            if self.bytes_serial[0] == 0x59 and self.bytes_serial[1] == 0x59: # 0x59 is 'Y'
                self.distance = self.bytes_serial[2] + self.bytes_serial[3]*256
                print(str(self.distance))
                msg.distance = float(self.distance)
                self.ser.reset_input_buffer()

            if self.bytes_serial[0] == "Y" and self.bytes_serial[1] == "Y":
                self.distL = int(self.bytes_serial[2].encode("hex"), 16)
                self.distH = int(self.bytes_serial[3].encode("hex"), 16)
                self.distance = self.distL + self.distH*256
                print(str(self.distance))
                msg.distance = float(self.distance)
                self.ser.reset_input_buffer()


        self.lidar_data.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    ##try:
    ##    if self.ser.isOpen() == False:
    ##        self.ser.open()
    ##    publish_lidar_data()
    ##except KeyboardInterrupt:   # Ctrl+C
    ##    if self.ser != None:
    ##        self.ser.close()
