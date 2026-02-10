import rclpy								#ros2 library for python
from rclpy.node import Node					#same as above
from sensor_msgs.msg import Joy				#importing the Joy interface that was downloaded in the joy package
import time									# importing time
import pigpio								# gpio library
import os
os.system("sudo pigpiod")
#import RPi.GPIO as GPIO
# time.sleep(1)

LED_pin = 26
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(LED_pin, GPIO.OUT)

class LED_Modulation(Node):
    def __init__(self):
		# defining the LED Pin
        self.LED_state = True
        self.LED_pin1 = 26
        self.LED_pin2 = 19
        self.pi = pigpio.pi()

        #Initializing the node and nameing it "led" 
        super().__init__("led")

        #Subscribing to the /joy topic with is the controller read information
        self.subscriber = self.create_subscription(Joy, "joy", self.callback_LED_input, 10)
        time.sleep(1)

    def callback_LED_input(self,msg):
        if msg.buttons[6] == 1:
            self.LED_state = not self.LED_state
            time.sleep(1)

        if self.LED_state is False:
            #GPIO.output(LED_pin, GPIO.HIGH)
            self.pi.write(self.LED_pin1, 1)
            self.pi.write(self.LED_pin2, 1)
            self.get_logger().info("LED should be off")
        else:
            self.pi.write(self.LED_pin1, 0)
            self.pi.write(self.LED_pin2, 0)
def main(args=None):
    rclpy.init(args=args)
    node = LED_Modulation()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
