from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/demo.launch.py'))
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
            'aruco_detection_node = camera_service.aruco_detection_node:main',
            'object_spawner_node = camera_service.object_spawner_template:main'
        ],
    },
)
