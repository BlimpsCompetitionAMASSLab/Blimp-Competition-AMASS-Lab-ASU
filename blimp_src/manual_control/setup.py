from setuptools import find_packages, setup

package_name = 'manual_control'

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
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_to_esc = manual_control.joy_to_esc_input:main",
            "manual_esc = manual_control.manual_esc_control:main",
            "manual_bridge = manual_control.manual_bridge:main"
        ],
    },
)
