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
