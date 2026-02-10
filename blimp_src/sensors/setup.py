from setuptools import find_packages, setup

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='orangepi',
    maintainer_email='orangepi@orangepi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "read_altitude = sensors.barometer:main",
            "read_bno055 = sensors.BNO055:main",
            "read_bno085 = sensors.BNO085:main",
            "read_lidar = sensors.Lidar:main",
            "read_sonar = sensors.Sonar:main",
            "LED_modulation = sensors.LED:main",
        ],
    },
)
