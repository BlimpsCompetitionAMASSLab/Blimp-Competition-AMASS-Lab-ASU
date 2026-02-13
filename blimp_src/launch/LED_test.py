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

        Node(
			package ='sensors',
			executable='LED_modulation',
			name='LED',
		),
		        # Launching ESC Driver
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver',
			parameters = [{
				"MAC":"68:6C:E6:73:04:62"
			}]
		),
		Node(
			package='manual_control',
			executable='joy_to_esc',
			name='joy_to_esc',
			parameters = [{
				"Klm": 1.0,
				"Krm": 1.0,
			}]
        ),
				Node(
			package='controls',
			executable='mode_switch',
			name='mode_switcher',
        ),
				Node(
			package='sensors_cpp',
			executable='F_to_Esc',
			name='force_to_esc',
        )
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()