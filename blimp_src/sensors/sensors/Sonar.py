import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from blimp_interfaces.msg import LidarData

GPIO.setmode(GPIO.BCM)

GPIO_TRIGGER = 4
GPIO_ECHO = 27

GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar_node')
        self.sonar_data = self.create_publisher(LidarData, "sonar_data", 10)
        self.create_timer(0.5, self.publish_sonar_data)

    def publish_sonar_data(self):
        msg = LidarData()
        GPIO.output(GPIO_TRIGGER, True)
        GPIO.output(GPIO_TRIGGER, False)
        self.StartTime = time.time()
        self.StopTime = time.time()

        while GPIO.input(GPIO_ECHO) == 0:
            self.StartTime = time.time()
#            print("GPIO_Echo is 0")
        
        while GPIO.input(GPIO_ECHO) == 1:
            self.StopTime = time.time()
        
        self.TimeElapsed = self.StopTime - self.StartTime
        self.distance = (self.TimeElapsed*34300) / 2

        msg.distance = self.distance
        print(self.distance)
 #       print("Shoulld be collecting data")

        self.sonar_data.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SonarNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

