import rclpy									#package for ros2 in python
from rclpy.node import Node						#samsies
from blimp_interfaces.msg import EscInput 		#custum interface for Esc inputs
from blimp_interfaces.msg import CameraCoord	#custom interface for camera coordinates
from scipy.optimize import lsq_linear			#least squares function from scipy library
import numpy as np								#matrix library


class BomberNode(Node):
	def __init__(self):

		super().__init__("Bomber_node") #staring the node as Bomber_node

		# declareing a parameter for proportinal control
		self.declare_parameter('k',1.0) #kg/m^3
		self.k = self.get_parameter('k').value

		#creating a subscription to the cam_data topic and giving the data to the main callback
		self.subscriber = self.create_subscription(
			CameraCoord, "cam_data", self.callback_bomber, 3
		)

		#Center of the camera
		self.x_cntr = 320
		self.y_cntr = 240
		
		# creatin a publisher to publish the Esc_inputs
		self.publisher = self.create_publisher(EscInput, "ESC_Bomber_input", 10)
		self.get_logger().info("Bombing civilians")

	def callback_bomber(self,msg):
		#pwm_l is motor A:
        #pwm_r is motor B:
		#pwm_d is motor C:
        # top of camera needs to be pointing towards A aka pwm_l
		#getting the x and y position of the camera data
		x = msg.position[0]
		y = msg.position[1]

		#determining the position of the detection relative to the center of the screen
		v = np.array([x-self.x_cntr, self.y_cntr - y, 0])
		
		# matrix that show the direction that all of the forces are pushing relative to the center of the camera
		M = np.array([[0,0.866025,-0.866025],[1,-0.5,-0.5],[0,0,0]])

		# using a bounded least squares fit to determin the necessary inputs for each motor
		# least squares allows us to take the inverse of a non-independent matrix matrix to find X
		# Mx = b  with least squares it allows x =bA^-1 with a non-independent matrix.
		# look it up for more info. thats like verry basic explanation. 
		F = lsq_linear(M, v, bounds=(0, np.inf)).x

		# putting the info into the msg and publishing it to the /ESC_Bomber_input topic
		msg2 = EscInput()
		#self.get_logger().info(str(F[0]))
		msg2.pwm_l = float(F[0])*self.k + 1050.0
		msg2.pwm_r = float(F[1])*self.k + 1050.0
		msg2.pwm_u = 1050.0
		msg2.pwm_d = float(F[2])*self.k + 1050.0
		msg2.esc_pins = [5,6,13,26]

		## self.get_logger().info("A: " + str(msg2.pwm_l) + " B: " + str(msg2.pwm_r) + " C: " + str(msg2.pwm_d))
		self.publisher.publish(msg2)

def main(args=None):
	rclpy.init(args=args)
	node = BomberNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
