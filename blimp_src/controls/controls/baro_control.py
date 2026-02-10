import rclpy								#ros2 package for python
from rclpy.node import Node					#samsies
from blimp_interfaces.msg import BaroData	#Custom interface for barometer data
from blimp_interfaces.msg import EscInput	#custom Interface for Esc inputs

class BaroNode(Node):
	def __init__(self):

		super().__init__("Baro_node") # initilize the node as Baro_node
		
		# declaring parameters for the proportional controler (kpb) and the desired height (hight)
		# defaul values are 0.0
		self.declare_parameter('kpb',0.0) #kg/m^3
		self.declare_parameter('height',0.0) #kg/m^3

		# getting the parameter values from the launch file
		self.kpb = self.get_parameter('kpb').value
		self.height_goal = self.get_parameter('height').value

		# creating a subscription for the baraometer data
		# creating a publisher for to output the motor control
		self.subscriber = self.create_subscription(BaroData,"barometer_data", self.callback_baro_control, 10)
		self.publisher = self.create_publisher(EscInput, "ESC_Baro_input", 10)
		
		#get_logger to show that the node is working
		self.get_logger().info("barometer control has started")

	def callback_baro_control(self,msg):
		# this node will run every time that a baro reading is sent over the barometer_data topic
		# Grabing the height from the topic
		height = msg.height
		#creating the msg structure for the publisher with the type from EscInput
		msg2 = EscInput()

		# checkin if the read height is less than the goal height
		# if so then the we will calculate an error and the up motor will turn on
		if height < self.height_goal:
			msg2.pwm_l = 1500.0
			msg2.pwm_r = 1500.0
			# error calulation for the up motor. tune kpb for the desired response
			msg2.pwm_d = 1500.0 + abs(height-self.height_goal)*self.kpb
			# msg2.pwm_u = 1050.0
		# if the bomber is heigher than the goal height do nothing
		else:
			msg2.pwm_l = 1500.0
			msg2.pwm_r = 1500.0
			# msg2.pwm_u = 1050.0
			msg2.pwm_d = 1500.0
		# inputing the pins and sending the msg over the topic /ESC_baro_input
		msg2.esc_pins = [5,6,13]
		self.publisher.publish(msg2)
		self.get_logger().info("baro: " + str(height)+ " U: "+ str(msg2.pwm_d))
		
		
def main(args=None):
	rclpy.init(args=args)
	node = BaroNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
