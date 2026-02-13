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
        # Launching camera node

        # Launching Sonar
		Node(
			package='sensors',
			executable='read_sonar',
			name='sonar',
        ),
        # Launching Lidar
		Node (
			package='sensors',
			executable='read_lidar',
			name='lidar',
		),
	# Launching Camera
		Node (
			package='sensors_cpp',
			executable='old_cam',
			name='camera',
		),

        # Launching Force to ESC node

        # Launching inverse kinemtic model

        # Launching ESC Driver

        # Launching servo node for open/close net
		Node(
			package='controls',
			name='net_servo',
			executable='net_servo',
		),
        # Launching barometer

        # Launching IMU

		# Node(
		# 	package = 'sensors',
		# 	name = 'record_data',
		# 	executable = 'record_data',
		# 	parameters = [{"file_name":"pi_DBlue_4"}]
		# ),
	#	Node(
	#		package='sensors',
	#		name='sender_data',
	#		executable='sender'
	#	),
		# Node(
		# 	package='sensors',
		# 	name='reciever',
		# 	executable='recieve_data',
		# 	parameters = [{
		# 		"file_name": "test_data_1"
		# 	}]
		# ),
        # Launching Mode switchter to switch between manual and autonomous
		Node(
			package='controls',
			executable='mode_switch',
			name='mode_switcher',
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

