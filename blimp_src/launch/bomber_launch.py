from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		#Outside dist packages
		Node(
			package='controls',
			name='esc_motor_driver',
			executable='esc_driver'
		),
		Node(
			package='controls',
			name='random_walk_node',
			executable='random_walk'
		),
		Node(
			package='controls',
			name='Obomba',
			executable='bomber_cntrl',
			parameters = [{
				"k": 0.5,
			}]
		),
		Node(
			package='controls',
			name='mux',
			executable='mux',
		),
		Node(
			package='controls',
			name='baro_cntrl',
			executable='baro_cntrl',
			parameters = [{
				"kpb": 900,
				"height": 3.0,
			}]
		),
		Node(
			package='controls',
			name='net_servo',
			executable='net_servo'
		),
		Node(
			package='sensors_cpp',
			executable='balloon_detect_cpp',
			name='balloon_detection',
        ),
		Node(
			package='sensors',
			name='read_altitude',
			executable='read_altitude',
			parameters = [{
				"sea_level_pressure": 1016.9
			}]
		),
		
	])
def main(args=None):
	generate_launch_description()

if __name__ == "__main__":
	main()
