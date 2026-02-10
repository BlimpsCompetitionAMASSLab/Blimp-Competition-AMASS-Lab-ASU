import board
import time
import pigpio
import os
os.system ("sudo pigpiod")
time.sleep(1)


# max_servofq = 2500
# min_servofq = 500
servofq = 500

pi = pigpio.pi()

c = 0

while True:

    pi.set_servo_pulsewidth(17,servofq)

    time.sleep(1)
    if servofq == 2500:
        c = 1
    elif servofq == 500:
        c = 0

    if c == 0:
        servofq = servofq + 100
    elif c == 1:
        servofq = servofq - 100
