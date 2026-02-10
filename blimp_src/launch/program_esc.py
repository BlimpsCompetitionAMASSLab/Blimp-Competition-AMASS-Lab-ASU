import os                   #importing os library so as to communicate with the system
import time                 #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod")  #Launching GPIO library
time.sleep(1)               # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio               #importing GPIO library

#This file is used to program new esc's or reprogram old ones to new battery or motor specs
#When you run this hit enter after every Beep. 

ESC=6  #Connect the ESC in this GPIO pin. Currently the up motor pin. can change in the future.
#current pin options
# 5
# 6
# 13
# 26
pi = pigpio.pi();

#Each letter corresponds to a throttle value. These values are specified in the ESC user manual.
# To select different settings change the letter value in the code below
H = 1800
L = 1200
M = 1500


#This is the starting value to enter programing mode.
#  
pi.set_servo_pulsewidth(ESC, H)
input("When ESC power input is connected wait for sound a press enter")


# Here the Medium value is set to go to the next programing setting. Do not change this value
pi.set_servo_pulsewidth(ESC, M)
x = input("When you hear one beep hit enter")


# Now this is the value to actually select the setting this is the value you want to change for the type of battery
# Settings Options:
# H: 3S Li-po Battery
# L: 2S Li-po Battery
# 70% option: check Tech sheet for more info
pi.set_servo_pulsewidth(ESC,L)
input("If selection was made hit enter")


#Setting the value back to medium to go to the next menu option
pi.set_servo_pulsewidth(ESC, M)
x = input("When you hear two beeps hit enter")

# The value below is what you will want to change this one effects Braking of the motor
#Settings Option:
# H: Brakeing off
# L: Brakeing on
pi.set_servo_pulsewidth(ESC,L)
input("If selection was made hit enter")


#Setting the value back to medium to go to the next menue option
pi.set_servo_pulsewidth(ESC, M)
x = input("When you hear three beeps hit enter")

# The below menue option is for the timing of the motor
# H: 4 or more poles timing
# L: 2 pole timeing
pi.set_servo_pulsewidth(ESC,H)
input("If selection was made hit enter")

#Setting the value back to medium to go to the next menue option
pi.set_servo_pulsewidth(ESC, M)
x = input("When you hear four beeps hit enter")


# This menue option is for throttle input range
# H: throttle range 1100 pwm to 1900 pwm
# L: throttle range 1200 pwm to 1800 pwm
pi.set_servo_pulsewidth(ESC,H)
input("If selection was made hit enter")

#stoping the program
pi.set_servo_pulsewidth(ESC, M)
print("Disconnect ESC from power. Reconnect and ESC can now be used!!")
