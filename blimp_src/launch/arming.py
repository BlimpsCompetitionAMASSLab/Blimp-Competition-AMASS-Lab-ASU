import pigpio
import time
import os
os.system("sudo pigpiod")
time.sleep(1)
p1 = 13
p2 = 6
p3 = 5
p4 = 26

pi = pigpio.pi()
h = 2000
l  = 700
pi.set_servo_pulsewidth(p1,0)
time.sleep(1)
pi.set_servo_pulsewidth(p1,h)
time.sleep(1)
pi.set_servo_pulsewidth(p1,l)
time.sleep(1)
pi.set_servo_pulsewidth(p2,0)
time.sleep(1)
pi.set_servo_pulsewidth(p2,h)
time.sleep(1)
pi.set_servo_pulsewidth(p2,l)
time.sleep(1)
pi.set_servo_pulsewidth(p3,0)
time.sleep(1)
pi.set_servo_pulsewidth(p3,h)
time.sleep(1)
pi.set_servo_pulsewidth(p3,l)
time.sleep(1)
pi.set_servo_pulsewidth(p4,0)
time.sleep(1)
pi.set_servo_pulsewidth(p4,h)
time.sleep(1)
pi.set_servo_pulsewidth(p4,l)
time.sleep(1)
