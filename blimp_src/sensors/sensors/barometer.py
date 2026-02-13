# importing the libraries
import rclpy #for ros2 to work in python
from rclpy.node import Node #same as above
import time #for reading and manipulating time within the file
import board #for commuication between the board and the gpio pins
import adafruit_bmp3xx # libraries for barometer, adafruit_bmp3xx
from blimp_interfaces.msg import BaroData #custom message for the barometer data

class BarometerNode(Node): #Creating a Node

        def __init__(self):
                super().__init__('barometer_node') #initilizig the ros2 node and nameing it "barometer_node"
                #creating a publisher to publish the information read from the barometer.
                # BaroData is the msg type and "Barometer_data" is the topic name
                self.barometer_data = self.create_publisher(BaroData,"barometer_data",10)

                #starting i2c communications between the barometer and the program
                #self.sensors is the barometer object that we will read the data from
                self.i2c = board.I2C()
                self.sensor = adafruit_bmp3xx.BMP3XX_I2C(self.i2c)

                #declaring a parameter named Sea level pressure for the baro. default value is 999.323...
                # we then get the parameter from the launch file and set is as a global variable
                self.declare_parameter('sea_level_pressure', 999.3233801073472)
                self.sea_level_pressure = self.get_parameter('sea_level_pressure').value

                #creating a timer so the baro will publish data every 0.1 seconds
                self.create_timer(0.1,self.publish_barometer_data)

                #using the obtained sealevel pressure and setting the baro value to that
                self.sensor.sea_level_pressure = self.sea_level_pressure
                #sole initlization of variables
                self.height_init =0.0
                self.c = 0

                # get_logger to know that the node is running. a heart for love <3
                self.get_logger().info("baro has started <3")
                

        def publish_barometer_data(self):
                #defining the msg type that will be published
                msg = BaroData()

                #try except is to check if the baro is running or not. if not it should set the height to 10 so the balloon will go down
                try:
                        #reading the initial height of the barometer and saveing it into a variable
                        # if statement should only run once and never again. 
                        if self.c == 0:
                                self.height_init = self.sensor.altitude
                                self.c = self.c + 1

                        #reading the height from the barometer and subtracting the initial height. 
                        # this proccess will give us the realive height. Using this method balloon should generally start on the floor when ros2 is started.
                        msg.height = float(self.sensor.altitude) - self.height_init
#                        print(msg.height)
                except: 
                        # if an error is caught this will print that one is found and hopefully fly to the floor.
                        self.get_logger().info("barometer offffffff whyyyyyyyy")
                        msg.height = 10.0
                
                # publishing the barometer data.
                self.barometer_data.publish(msg)

#stuff every ros2 node needs to have to run. dont really need to change this
def main(args=None):
        rclpy.init(args=args)
        #assigning node as the BarometerNode class
        node = BarometerNode()
        rclpy.spin(node)
        rclpy.shutdown()

if __name__ == '__main__': #this allows us to run script from terminal directly
        main()
