# Copyright 2023 ROS Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


"""
This load_yaml function returns a .yaml file content, given the ROS2 Package name and
relative filepath to that package.
"""

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():

    launch_description = LaunchDescription()
  
    # aruco_detection = Node(
    #     package="camera_service",
    #     executable="aruco_detection_node",
    #     name="aruco_detection_node",
    #     output="log"
    # )
    # # Using this example, create the rest of the laumch node yourself. 

    # launch_description.add_action(aruco_detection)

    object_spawner = Node(
        package='camera_service',
        executable='object_spawner_node',
        name='object_spawner_node',
        output='log'
    )

    launch_description.add_action(object_spawner)

    rviz_config_dir = os.path.join(
    get_package_share_directory('camera_service'),  # Thư mục share của package
    'launch',                                       # Thư mục chứa file rviz
    'practical9.rviz'                               # Tên file rviz
)
    
    run_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='log',
        arguments=rviz_config_dir
    )

    launch_description.add_action(run_rviz2)
 

    return launch_description