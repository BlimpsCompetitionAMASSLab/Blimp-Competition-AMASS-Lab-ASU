import socket
import pygame
import time

# Client setup
UDP_IP = "192.168.137.64"  # Replace with your server's IP address
UDP_PORT = 8000

pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()

if joystick_count == 0:
    print("No Xbox controller detected.")
    pygame.quit()
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Create a UDP socket

try:
    while True:
        pygame.event.pump()
        x_axis = joystick.get_axis(0)

        pwm_signal = int(((x_axis + 1) / 2) * (1900 - 1050) + 1050)
        send_time = time.time()
        
        message = f"{pwm_signal}|{send_time}".encode()
        sock.sendto(message, (UDP_IP, UDP_PORT))
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Client is shutting down.")

finally:
    pygame.quit()
    sock.close()
