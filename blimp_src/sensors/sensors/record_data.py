import rclpy
from rclpy.node import Node
import math
import time
#from std_msgs.msg import Float32MultiArray #also have to add dependcy package into package.xml
from blimp_interfaces.msg import EscInput
from blimp_interfaces.msg import CameraCoord
from sensor_msgs.msg import Joy

class RecordDataNode(Node): #Creating a Node
    
	def __init__(self): #initiating node
		self.Manual_mode  = True
		self.butt = 0
		self.cam = [0,0]
		self.motor = [0.0,0.0,0.0,0.0]
		super().__init__('recording_node') #naming node 'recording_data'
		self.declare_parameter('file_name','test_data')

		self.subscriber_esc = self.create_subscription(	EscInput, "ESC_input", self.callback_record_esc, 10)
		#self.create_timer(0.1, self.callback_record_esc)
		self.start_time = time.time()
		self.get_logger().info("Data is starting to record")

	def callback_record_esc(self,msg):
		dt = time.time() - self.start_time
		fn = self.get_parameter('file_name').get_parameter_value().string_value
		f = open(str(fn), 'a')
		f.write(" L Motor: {}".format(msg.pwm_l) + " " +
			"R Motor: {}".format(msg.pwm_r) + " " + "U Motor: {}".format(msg.pwm_u) + " " + 
			"D Motor: {}".format(msg.pwm_d) + " " + "time: {}".format(dt) + "\n")
		f.close()
def main(args=None):
	rclpy.init(args=args)
	node = RecordDataNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
	main()
