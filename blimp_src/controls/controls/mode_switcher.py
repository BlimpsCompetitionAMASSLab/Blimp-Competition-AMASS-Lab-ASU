import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput 
from blimp_interfaces.msg import CameraCoord
from sensor_msgs.msg import Joy
import time

class BalloonPI(Node):
	def __init__(self):
		# program will always start with manual mode being true
		self.Manual_mode = True
		# pins for the esc and other initialization stuff
		self.manual_pins = [5,6,13]
		self.manual_L = 0
		self.manual_R = 0
		#self.manual_U = 0
		self.manual_D = 0

		self.camera_pins = self.manual_pins
		self.camera_L = 0
		self.camera_R = 0
		#self.camera_U = 0
		self.camera_D = 0

		super().__init__("mode_switcher")# initializing the mode swither
		
		#subscribing to the manual and autonmy esc input topics 
		self.manual_subscriber = self.create_subscription(
			EscInput, "ESC_Manual_input", self.callback_manual, 10
		)
		
		self.camera_subscriber = self.create_subscription(
			EscInput, "ESC_balloon_input", self.callback_camera, 10
		)
		#self.camera_subscriber = self.create_subscription(EscInput, "ESC_extremum_seeking_input", self.callback_camera, 10)

		#sub to joy for switching the modes either autonmy or manual
		self.subscriber = self.create_subscription(
			Joy, "joy", self.callback_switch_mode, 10
		)
		
		#to publish to the esc driver
		self.publisher = self.create_publisher(EscInput, "ESC_input", 10)
		
		self.get_logger().info("Started pi control for balloon detection.")
		
	def callback_camera(self,msg):
		# asssigning the values from the msg to a global variable
		self.camera_pins = msg.esc_pins
		self.camera_L = float(msg.pwm_l)
		self.camera_R = float(msg.pwm_r)
		#self.camera_U = float(msg.pwm_u)
		self.camera_D = float(msg.pwm_d)
		
	def callback_manual(self,msg):
		# asssigning the values from the msg to a global variable
		self.manual_pins = msg.esc_pins
		self.manual_L = float(msg.pwm_l)
		self.manual_R = float(msg.pwm_r)
		#self.manual_U = float(msg.pwm_u)
		self.manual_D = float(msg.pwm_d)
		

	def callback_switch_mode(self, msg):
		#defining the msg type
		msg2 = EscInput()
		
		#if the button is pressed the mode will change
		if msg.buttons[0]  == 1:
			self.Manual_mode = not self.Manual_mode
			time.sleep(2)
			self.get_logger().info("Manual Mode is " + str(self.Manual_mode))
			
		#depending on the mode different values will be sent
		if self.Manual_mode == True:
			msg2.esc_pins = self.manual_pins
			msg2.pwm_l = float(self.manual_L)
			msg2.pwm_r = float(self.manual_R)
			#msg2.pwm_u = float(self.manual_U)
			msg2.pwm_d = float(self.manual_D)
		
		elif self.Manual_mode == False:
			msg2.esc_pins = self.camera_pins
			msg2.pwm_l = float(self.camera_L)
			msg2.pwm_r = float(self.camera_R)
			#msg2.pwm_u = float(self.camera_U)
			msg2.pwm_d = float(self.camera_D)
		
		# publish the values
		self.publisher.publish(msg2)
			
		


def main(args=None):
	rclpy.init(args=args)
	node = BalloonPI()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
