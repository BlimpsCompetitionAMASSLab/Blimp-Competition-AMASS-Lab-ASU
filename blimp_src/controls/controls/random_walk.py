import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput 
import random

class RandWalkNode(Node):
	def __init__(self):

		super().__init__("random_walk_node")
		self.publisher = self.create_publisher(EscInput, "ESC_random_input", 10)
		self.get_logger().info("Walking is randomized")
		self.create_timer(2,self.callback_random_walk)

	def callback_random_walk(self):
		# generating a random interger to choos the motor
		motor = random.randint(0,2)
		# generating a random pwm
		pwm = float(random.randint(1100, 1300))
		msg = EscInput()
		#pwm_l is motor A:
        #pwm_r is motor B:
		#pwm_d is motor C:
        #See documentation if scott ever gets to that gl brother.
		# assign the rand motor to the rand pwm
		if motor == 0:
			msg.pwm_l = pwm
			msg.pwm_r = 1050.0
			msg.pwm_u = 1050.0
			msg.pwm_d = 1050.0
		elif motor == 1:
			msg.pwm_l = 1050.0
			msg.pwm_r = pwm
			msg.pwm_u = 1050.0
			msg.pwm_d = 1050.0
		else:
			msg.pwm_l = 1050.0
			msg.pwm_r = 1050.0
			msg.pwm_u = 1050.0
			msg.pwm_d = pwm
		msg.esc_pins = [5,6,13,26]
		
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	node = RandWalkNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
