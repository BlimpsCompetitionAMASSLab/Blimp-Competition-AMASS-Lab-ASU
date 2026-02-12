from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
        # Lauching Xbox Controller
		Node(
			package='joy', # Package the node is in 
			executable='game_controller_node', # Executable found in setup.py 
			name='joy_con', # Name can be whatever you want
        ),
		# Launching stick input to motor input node
		Node(
			package='manual_control',
			executable='joy_to_esc',
			name='joy_to_esc',
			parameters = [{
				"Klm": 1.0,
				"Krm": 1.0,
			}]
        ),
        # Launching ESC Driver
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver',
			parameters = [{
				"MAC":"68:6C:E6:84:3D:AA" # Black Controller
			}]
		),
        # Barometer Control Node:
		Node(
			package='controls',
			name='baro_cntrl',
			executable='baro_cntrl',
			parameters = [{
				"kpb": 150.0,
				"height": 6.5,
			}]
		),
        # Launching barometer
		Node(
			package='sensors',
			name='read_altitude',
			executable='read_altitude',
			parameters = [{
				"sea_level_pressure": 1016.0
			}]
		),
        # Launching Mode switchter to switch between manual and autonomous
		Node(
			package='controls',
			executable='rudolph_mode_switch',
			name='rudolph_mode_switcher',
        ),
		Node(
			package ='sensors',
			executable='LED_modulation',
			name='LED',
		)
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()
