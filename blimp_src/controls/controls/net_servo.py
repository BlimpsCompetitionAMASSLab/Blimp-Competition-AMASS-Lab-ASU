import rclpy						#ros2 package for python
from rclpy.node import Node			#samsies
from sensor_msgs.msg import Joy		#interface that the controller script publishes with
from blimp_interfaces.msg import LidarData
from blimp_interfaces.msg import CameraCoord
from blimp_interfaces.msg import Bool
import pigpio						#library for the gpio pins
import os							#allows for usage of cmd line inputs in the python script
import time							#time
import numpy as np

#starting the pigpio daemon. need the sleep after because it needs time to process
os.system("sudo pigpiod")
time.sleep(1)


class NetServo(Node):
	def __init__(self):
		super().__init__("net_servo") #initializing the node with the name net_servo

		#assigning the class object to self.pi
		self.pi = pigpio.pi()
		self.dist_lidar_old = 0
		self.position_old_x = 0
		self.position_old_y = 0
		self.dist_sonar = 0
		self.goal_flag = False

		#the pin that the servo will use for pwm signal
		self.pin_net = 17
		# starting pos of net will be open
		self.net = False

		# ALso want to publish back to the camera in some case:
		self.publisher= self.create_publisher(
			Bool, "net_flag", 10
		)

		#subscribing to the /joy topic to get the button inputs 
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_net, 10
		)

		self.subscriber = self.create_subscription(
			LidarData, "lidar_data", self.callback_net_lidar, 10
		)

		self.subscriber = self.create_subscription(
			LidarData, "sonar_data", self.callback_net_sonar, 10
		)

		self.subscriber = self.create_subscription(
			CameraCoord, "cam_data", self.callback_net_camera, 10
		)

		self.subscriber = self.create_subscription(
			Bool, "cam_flag", self.callback_cam_flag, 10
		)

		#getlogger to show node has started
		self.get_logger().info("Net servo has started")

	def callback_cam_flag(self, flag_msg):
		self.goal_flag = flag_msg.flag
	def callback_net_lidar(self, msg):
		dist_lidar = msg.distance
		#print(dist_lidar)
		if ((self.dist_lidar_old - dist_lidar) > 10 and dist_lidar < 30 and dist_lidar != 0):
			self.net = True
			self.actuate_net()
		self.dist_lidar_old = dist_lidar

	def callback_net_sonar(self, msg):
		self.dist_sonar = msg.distance
		if ((self.dist_sonar < 10) and (self.goal_flag is True)):
			self.net = False
			self.actuate_net()
		self.goal_flag = False

	def callback_net_camera(self, msg): #call back only runs when we see shit
		if (msg.position[0] - self.position_old_x != 0) and (msg.position[1] - self.position_old_y !=0):
			self.get_logger().info("Seeing something")

	def callback_net(self, msg):
		# if statement will check if the B button has been pressed
		if msg.buttons[1]  == 1:
			#changes the boolian to the opposit of itself
			self.net = not self.net
			self.actuate_net()
			#sleep to give you some time to unpress the button
			time.sleep(1)
			#showing that the state that the net was changed to
			self.get_logger().info("Net closed is " + str(self.net))
		
		# depending on the state of the net it will either open or close
		# False is open True is closed
	
	def actuate_net(self):
		msg = Bool()
		msg.flag = self.net  # Set the 'flag' field of the Bool message
		self.publisher.publish(msg)
		if self.net is False:
			self.pi.set_servo_pulsewidth(self.pin_net, 500)
		elif self.net is True:
			self.pi.set_servo_pulsewidth(self.pin_net, 2500)



def main(args=None):
	rclpy.init(args=args)
	node = NetServo()
	rclpy.spin(node)
	os.system("sudo killall pigpiod")
	rclpy.shutdown()

if __name__ == "__main__":
	main()
