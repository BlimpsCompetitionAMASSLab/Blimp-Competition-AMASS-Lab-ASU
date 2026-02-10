import rclpy								#ros2 library for python
from rclpy.node import Node					#same as above
from sensor_msgs.msg import Joy				#importing the Joy interface that was downloaded in the joy package
from blimp_interfaces.msg import EscInput	#importing the custom EscInput interface
import time									# importing time

class FixAxesNode(Node):
	def __init__(self):
		# defining the pin numbers
		self.ESC_pin1 = 5
		self.ESC_pin2 = 6
		self.ESC_pin3 = 13
		#self.ESC_pin4 = 26
		self.joy_time = 0
		
		#Initializing the node and nameing it "joy_to_esc" 
		super().__init__("joy_to_esc")

		# sometime the motors are not the same so we have variables to multiply the inputs to try and get them closser
		self.declare_parameter('Klm', 1.0)
		self.declare_parameter('Krm', 1.0)
		self.Klm = self.get_parameter('Klm').value
		self.Krm = self.get_parameter('Krm').value
		
		#Subscribing to the /joy topic with is the controller read infformation
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_manual_esc_input, 10
		)
		
		#setting up the publisher EscInput is the message type and ESC_Manual_input is the topic name
		self.publisher = self.create_publisher(EscInput, "ESC_Manual_input", 10)	
		#get_logger to show that the node has started
		self.get_logger().info("Data is being sent to the ESC node")

	def callback_manual_esc_input(self, msg):
		#so the idea rn is the left joy stick moves on the xbox controller moves the drone forwards.
		#to turn the blimp the left and right triggers are used
		# the right joy stick on the motor is used for going up or down
		#
		#   				Table of Analog Inputs from /joy
		#=========================================================================
		#    L = Left dir           R = Right Dir           M = middle pos 
		#           Unp = unpressed trig   P = pressed Trig Value
		#=========================================================================
		# axes array 	  input		 Max value	Min value   middle or
		#   index        on cntrl	 & dirctn   & direction equlibrium value
		# -----------    --------    -------    --------    -------
		# msg.axes[0]:   Left Joy    L = 1.0    R = -1.0    M = 0.0
		# msg.axes[1]:   Left Joy    U = 1.0    D = -1.0    M = 0.0
		# msg.axes[2]:   Right Joy   U = 1.0    D = -1.0    M = 0.0
		# msg.axes[3]:   Right Trig  UnP = 1.0  P = -1.0
		# msg.axes[4]:   Right Joy   L = 0.0    R = -1      M = -0.5
		# msg.axes[5]:   Left Trig   UnP = 0.0  P = -1.0
		
		#setting trig values will range from 0 to 50
		RTrim = -450*(msg.axes[5])
		LTrim = 450+((0-450)/(2))*(msg.axes[3]+1)

		# setting the forrward value that will go to each motor will range from 0 to 50
		F = 1950+((1050-1950)/(-2))*(msg.axes[1]-1)

		# Lm is the Left motor and RM is the right motor
		# will add the forward value to the trim value. will range from 0 to 100
		LM = (F*self.Klm) + LTrim - RTrim
		RM = (F*self.Krm) + RTrim - LTrim
		DM = 1950+((1050-1950)/(-2))*(msg.axes[2]-1)
		# # checking bounds
		# if LM < 0:
		# 	LM = 0
		# elif RM < 0:
		# 	RM = 0

		# if LM > 100:
		# 	LM = 100
		# elif RM > 100:
		# 	RM = 100

		# # dividing the 
		# if msg.axes[2] > 0.05:
		# 	UM = abs(msg.axes[2])*100
		# 	DM = 0
		# elif msg.axes[2] < -0.05:
		# 	DM = abs(msg.axes[2])*100
		# 	UM = 0
		# else:
		# 	UM = 0
		# 	DM = 0
		
		# turning the values from 0 - 100 into 1050 to 1900 for the pwm input
		# LM_pwm = self.control_to_esc_input(LM)
		# UM_pwm = self.control_to_esc_input(UM)
		# DM_pwm = self.control_to_esc_input(DM)
		# RM_pwm = self.control_to_esc_input(RM)
		LM_pwm = LM
		RM_pwm = RM
		DM_pwm = DM
		
		# defining msg type to be the EscInput 
		msg2 = EscInput()
		# inputing the pin values and pwm values into the msg
		msg2.esc_pins = [self.ESC_pin1, self.ESC_pin2, self.ESC_pin3]
		msg2.pwm_l = LM_pwm
		msg2.pwm_r = RM_pwm
		#msg2.pwm_u = UM_pwm
		msg2.pwm_d = DM_pwm

		#publishing the msg
		self.publisher.publish(msg2)

		
	def control_to_esc_input(self, input):
		#linear interpolation for the 0 -100 into 1050 - 1900
		pwm = 1050 + (((input-0)*(1900-1050))/(100 - 0))
		#returns the pwm value
		return pwm

def main(args=None):
	rclpy.init(args=args)
	node = FixAxesNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
