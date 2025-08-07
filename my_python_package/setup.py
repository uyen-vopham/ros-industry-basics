from setuptools import find_packages, setup
import os

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/two_nodes.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uyen',
    maintainer_email='uyen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node_executable = my_python_package.my_node:main',
            'teleop_keyboard = my_python_package.my_teleop:main',
            'my_listener_node = my_python_package.my_listener:main',
            'my_talker_node = my_python_package.my_talker:main',
            'my_camera_node = my_python_package.my_camera:main',
            'simple_velocity_pub_node = my_python_package.custom_msg_publisher:main',
            'player1_node = my_python_package.player1:main',
            'simple_velocity_sub_node = my_python_package.custom_msg_subscriber:main',
            'player2_node = my_python_package.player2:main'
        ],
    },
)
