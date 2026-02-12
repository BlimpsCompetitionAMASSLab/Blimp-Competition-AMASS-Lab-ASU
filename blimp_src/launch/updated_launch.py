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
		Node(
			package='sensors_cpp',
			executable='old_cam',
			name='cam_node',
        ),
        # Launching Sonar
		# Node(
		# 	package='sensors',
		# 	executable='read_sonar',
		# 	name='sonar',
        # ),
        # Launching Lidar
		# Node (
		# 	package='sensors',
		# 	executable='read_lidar',
		# 	name='lidar',
		# ),
		# Launching PID Controler
		Node(
			package='sensors_cpp',
			executable='pi_controller',
			name='balloon_detect_PI',
			parameters = [{
				"iheight": 2.1, # Initial Height
				"kpx": 0.0003, # side mototrs - Proportional
				"kix":  0.0, # side motors - Integral
				"kpyu": 0.0, # up motor - Proportional
				"kpyd": 0.0015, # down motor - Proportional
				"kiy":  0.0, # up/down motor - Integral
				"kpb":  0.69  # barometer - Proportional
			}]
        ),
        # Launching Force to ESC node
		Node(
			package='sensors_cpp',
			executable='F_to_Esc',
			name='force_to_esc',
        ),
        # Launching inverse kinemtic model
		Node(
			package='sensors_cpp',
			executable='dynamic_model',
			name='inv_kine',
			parameters = [{
                            # Mass of Blimp - Negative Mass (from not beingn neutrally bouyant)                
				"buoyancy": ((0.460386) - (0*0.00))*9.81,
				"rho_air": 1.225 # Air density
			}]
        ),
        # Launching ESC Driver
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver',
			parameters = [{
				"MAC":"28:EA:0B:F5:F7:9D"
			}]
		),
        # Launching servo node for open/close net
		Node(
			package='controls',
			name='net_servo',
			executable='net_servo'
		),
        # Launching barometer
		Node(
			package='sensors',
			name='read_altitude',
			executable='read_altitude',
			parameters = [{
				"sea_level_pressure": 1017.0
			}]
		),
        # Launching IMU
		Node(
			package='sensors',
			name='read_imu',
			executable='read_bno085'
		),
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
		# Node(
		# 	package ='sensors',
		# 	executable='LED_modulation',
		# 	name='LED',
		# )
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()
