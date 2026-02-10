import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import EscInput
import time

class ManualBridgeNode(Node):
	def __init__(self):
		super().__init__("esc_input")
		
		self.subscriber = self.create_subscription(
			EscInput, "ESC_Manual_input", self.callback_manual_bridge, 10
		)
		
		self.publisher = self.create_publisher(EscInput, "ESC_input", 10)
		
		#self.time_publisher = self.create_publisher(Float64, "time", 10)
		
		
		self.get_logger().info("Data is being sent to the ESC node")

	def callback_manual_bridge(self, msg):
		msg2 = EscInput()
		msg2.esc_pins  = msg.esc_pins
		msg2.esc_pwm = msg.esc_pwm
		self.publisher.publish(msg2)

def main(args=None):
	rclpy.init(args=args)
	node = ManualBridgeNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
