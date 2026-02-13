from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		#Outside dist packages
		Node(
			package='joy',
			executable='game_controller_node',
			name='joy_con',
			#parameters = [{"autorepeat_rate": 10.0}]
        ),
		#Manual Control Package Excecutable
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
			package='sensors_cpp',
			executable='old_cam',
			name='old_cam_node',
        ),
		#Control Package Executables:
		Node(
			package='sensors_cpp',
			executable='ES_control',
			name='Extremum',
        ),

		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver',
			parameters = [{
				"MAC":"68:6C:E6:73:04:62"
			}]
		),
		Node(
			package='controls',
			executable='mode_switch',
			name='mode_switcher',
        )
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()