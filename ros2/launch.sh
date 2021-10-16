# Creating a launch file
mkdir launch && touch launch/turtlesim_mimic_launch.py
echo "from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])" > launch/turtlesim_mimic_launch.py
cd launch
ros2 launch turtlesim_mimic_launch.py
# if has package keywords
# reference to https://docs.ros.org/en/galactic/Tutorials/Creating-Your-First-ROS2-Package.html#createpkg
# ros2 launch <package_name> <launch_file_name> 
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
rqt_graph

# write a launch file: https://docs.ros.org/en/galactic/Tutorials/Launch-system.html
export work_root="~/work_root/ros2_tutorial"
export workspace="${work_root}/demo_lauch"
mkdir -p "${workspace}/src" && cd "${workspace}/src"
# cpp launch version:
ros2 pkg create cpp_my_demo_launch_package --build-type ament_cmake
cd cpp_my_demo_launch_package && mkdir launch && cd launch
echo "import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])" >> my_script_launch.py
# modify CMakeList.txt (path is  ${work_space}/src/cpp_my_demo_launch_package) according to https://docs.ros.org/en/galactic/Tutorials/Launch-system.html

# python launch version
ros2 pkg create py_my_demo_launch_package --build-type ament_python
cd py_my_demo_launch_package && mkdir launch && cd launch
echo "import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen',
            name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])" >> my_script_launch.py
# modify setup.py (path is  ${work_space}/src/py_my_demo_launch_package)according to https://docs.ros.org/en/galactic/Tutorials/Launch-system.html
# when modify setup.py do not forget to import os and glob which are buildin packages in python

# build
cd ${work_space} && colcon build
. install/setup.bash
export USER='your_defined_username'
ros2 launch py_my_demo_launch_package my_script_launch.py
