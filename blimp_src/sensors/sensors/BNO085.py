import rclpy 				#for ros2 to work in python
from rclpy.node import Node #same as above
import time
import board
import busio
import math as m
from adafruit_bno08x import (
	BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from blimp_interfaces.msg import ImuData #custom interface made for the imu
   

class ImuNode(Node): #Creating a Node

	def __init__(self): #initiating node
		super().__init__('imu_node') #naming node 'imu_node'
        # The BNO055 setup here is for i2c protocol. If you want to start using uart, you need to look it up.
		self.imu_data = self.create_publisher(ImuData,"imu_data",10) #Initializing publisher (message type,name,Qsize(some buffer thing:10 messages before it erases last one)S)
		i2c = busio.I2C(board.SCL, board.SDA)
		self.bno = BNO08X_I2C(i2c)
		self.bno.enable_feature(BNO_REPORT_LINEAR_ACCELERATION)
		self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
		self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
		self.c = 1
		self.create_timer(0.01, self.publish_imu_data) #calls function every 0.2 seconds
		
		self.get_logger().info("imu has Started")

		self.start_time = time.time()

	def publish_imu_data(self):
		msg = ImuData()
		
		try:
			accel_x, accel_y, accel_z = self.bno.linear_acceleration
			gyro_x, gyro_y, gyro_z = self.bno.gyro
			q1, q2, q3, q0 = self.bno.quaternion
			
			# Check if sensor returned valid data (not None)
			if None in [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, q0, q1, q2, q3]:
				return  # Skip this iteration if any value is None

			roll, pitch, yaw = self.quat2euler(q0, q1, q2, q3)
			
			if (self.c == 1):
				self.int_roll = roll
				self.int_pitch = pitch
				self.int_yaw = yaw
				self.c += 1
			roll -= self.int_roll
			pitch -= self.int_pitch
			yaw -= self.int_yaw

			
			msg.imu_lin_accel = [accel_x,accel_y,accel_z]
			msg.imu_gyro = [gyro_x,gyro_y,gyro_z]
			msg.imu_euler = [roll,pitch,yaw]
			print(msg.imu_euler)
			self.imu_data.publish(msg)
		except (IndexError, OSError, TypeError) as e:
			# Catch I2C communication errors and library bugs
			# This prevents the node from crashing
			pass
		
	def quat2euler(self, q0, q1, q2, q3):
		t0 = 2*(q0*q1 + q2*q3)
		t1 = 1 - 2*(q1*q1 + q2*q2)
		roll = m.degrees(m.atan2(t0,t1))
		t2 = 2*(q0*q2 - q3*q1)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch = m.degrees(m.asin(t2))
		t3 = 2*(q0*q3 + q1*q2)
		t4 = 1 - 2*(q2*q2 + q3*q3)
		yaw = m.degrees(m.atan2(t3,t4))
		return roll, pitch, yaw
        
def main(args=None):
	rclpy.init(args=args)
	node = ImuNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
	main()


