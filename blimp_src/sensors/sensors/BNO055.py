 # importing the libraries
import rclpy 				#for ros2 to work in python
from rclpy.node import Node #same as above
import time 				#for reading and manipulating time within the file
import board 				#for commuication between the board and the gpio pins
import busio 				#mor communication biz
import adafruit_bno055		#sensor library for the imu
import numpy as np 			#Matrix math library
import math					#math library
from blimp_interfaces.msg import ImuData #custom interface made for the imu


class ImuNode(Node): #Creating a Node

	def __init__(self): #initiating node
		super().__init__('imu_node') #naming node 'imu_node'
        # The BNO055 setup here is for i2c protocol. If you want to start using uart, you need to look it up.
        
		self.imu_data = self.create_publisher(ImuData,"imu_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
		self.sensor.mode = adafruit_bno055.M4G_MODE
		self.sensor.mode = adafruit_bno055.NDOF_MODE
		self.create_timer(0.01, self.publish_imu_data) #calls function every 0.2 seconds
		# initializing lists of saved variables and send variables
		self.prev_lin_accel = [0 ,0 ,0]
		self.prev_gyro = [0 ,0 ,0]
		self.prev_euler = [0 ,0 ,0]
		self.imu_lin_accel = [0 ,0 ,0]
		self.imu_gyro = [0 ,0 ,0]
		self.imu_euler = [0 ,0 ,0]
		self.get_logger().info("imu has Started")

		self.start_time = time.time()

	def publish_imu_data(self):
		msg = ImuData()
		lin_accel = self.sensor.linear_acceleration
		gyro = self.sensor.gyro
		euler = self.sensor.euler
		
		# Check accelerations for None and NaNs
		# this will iterate over the values in lin_accel
		# if a NaN or a None is found the value will be replaced with the last read non nan value
		for index, value in enumerate(lin_accel):
			if value is None or math.isnan(value):
				self.imu_lin_accel[index] = self.prev_lin_accel[index]
			else:
				self.imu_lin_accel[index] = value
				self.prev_lin_accel[index] = value

		# Check gyros for None and NaNs
		for index, value in enumerate(gyro):
			if value is None or math.isnan(value):
				self.imu_gyro[index] = self.prev_gyro[index]
			else:
				self.imu_gyro[index] = value
				self.prev_gyro[index] = value

		# Check Eulers for None and NaNs
		for index, value in enumerate(euler):
			if value is None or math.isnan(value):
				self.imu_euler[index] = self.prev_euler[index]
			else:
				self.imu_euler[index] = value
				self.prev_euler[index] = value
			
			self.imu_euler[0] = self.imu_euler[0]*(np.pi/180)
			self.imu_euler[1] = self.imu_euler[1]*(np.pi/180)
			self.imu_euler[2] = self.imu_euler[2]*(np.pi/180)

		#setting the msg values to the read accel, gyro, and euler variables
		msg.imu_lin_accel = self.imu_lin_accel
		msg.imu_gyro = self.imu_gyro
		msg.imu_euler = self.imu_euler

		#	self.get_logger().info(str(msg.imu_euler)) # Displays data on command line

		#publishes the read data
		self.imu_data.publish(msg)
        
def main(args=None):
	rclpy.init(args=args)
	node = ImuNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
	main()


