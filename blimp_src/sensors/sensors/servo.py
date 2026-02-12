import pigpio
import time

SERVO_PIN = 17  # GPIO pin connected to the servo

pi = pigpio.pi()  # Connect to pigpio daemon
if not pi.connected:
    print("Failed to connect to pigpio daemon")
    exit()

def move_servo_range(start=500, end=2000, step=100, delay=0.5):
    for pulse in range(start, end + 1, step):
        pi.set_servo_pulsewidth(SERVO_PIN, pulse)
        print(f"Moved to {pulse}µs")
        time.sleep(delay)

try:
    move_servo_range()
finally:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Turn off servo signal
    pi.stop()
