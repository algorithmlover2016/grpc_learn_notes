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

# Using ROS2 launch for large projects
prerequirest:
# reference to https://docs.ros.org/en/galactic/Tutorials/Tf2/Introduction-To-Tf2.html#intrototf2
apt-get install ros-galactic-turtle-tf2-py ros-galactic-tf2-tools
pip3 install transforms3d
# donot forget source /opt/ros/galactic/setup.bash to add search path
. /opt/ros/galactic/setup.bash
# reference to https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html
cd "${work_space}/src"
ros2 pkg create launch_tutorial --build-type ament_python
cd launch_tutorial && mkdir launch
cd launch
echo "import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
   turtlesim_world_1 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_1.launch.py'])
      )
   turtlesim_world_2 = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_world_2.launch.py'])
      )
   broadcaster_listener_nodes = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/broadcaster_listener.launch.py']),
      launch_arguments={'target_frame': 'carrot1'}.items(),
      )
   mimic_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/mimic.launch.py'])
      )
   fixed_frame_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/fixed_broadcaster.launch.py'])
      )
   rviz_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('launch_tutorial'), 'launch'),
         '/turtlesim_rviz.launch.py'])
      )

   return LaunchDescription([
      turtlesim_world_1,
      turtlesim_world_2,
      broadcaster_listener_nodes,
      mimic_node,
      fixed_frame_node,
      rviz_node
   ])" > launch_turtlesim.launch.py

echo "from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
   background_r_launch_arg = DeclareLaunchArgument(
      'background_r', default_value=TextSubstitution(text='0')
   )
   background_g_launch_arg = DeclareLaunchArgument(
      'background_g', default_value=TextSubstitution(text='84')
   )
   background_b_launch_arg = DeclareLaunchArgument(
      'background_b', default_value=TextSubstitution(text='122')
   )

   return LaunchDescription([
      background_r_launch_arg,
      background_g_launch_arg,
      background_b_launch_arg,
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         name='sim',
         parameters=[{
            'background_r': LaunchConfiguration('background_r'),
            'background_g': LaunchConfiguration('background_g'),
            'background_b': LaunchConfiguration('background_b'),
         }]
      ),
   ])" > turtlesim_world_1.launch.py

echo "import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('launch_tutorial'),
      'config',
      'turtlesim.yaml'
      )

   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='turtlesim_node',
         namespace='turtlesim2',
         name='sim',
         parameters=[config]
      )
   ])" > turtlesim_world_2.launch.py

cd "${work_space}/src"
cd launch_tutorial && mkdir config
cd config

echo "/turtlesim2/sim:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150" > turtlesim.yaml

# for wildards
echo "/**:
   ros__parameters:
      background_b: 255
      background_g: 86
      background_r: 150" >> turtlesim.yaml

cd ../launch
echo "import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
            get_package_share_directory('launch_tutorial'),
            'config',
            'turtlesim.yaml'
            )

    return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim',
                parameters=[config]
                )
    ])" > turtlesim_world_3.launch.py

echo "from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
         'target_frame', default_value='turtle1',
         description='Target frame name.'
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster1',
         parameters=[
            {'turtlename': 'turtle1'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_broadcaster',
         name='broadcaster2',
         parameters=[
            {'turtlename': 'turtle2'}
         ]
      ),
      Node(
         package='turtle_tf2_py',
         executable='turtle_tf2_listener',
         name='listener',
         parameters=[
            {'target_frame': LaunchConfiguration('target_frame')}
         ]
      ),
   ])" > broadcaster_listener.launch.py

echo "from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      Node(
         package='turtlesim',
         executable='mimic',
         name='mimic',
         remappings=[
            ('/input/pose', '/turtle2/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
         ]
      )
   ])" > mimic.launch.py

echo "import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
            get_package_share_directory('turtle_tf2_py'),
            'rviz',
            'turtle_rviz.rviz'
            )

    return LaunchDescription([
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config]
                )
    ])" > turtlesim_rviz.launch.py
echo "from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
      DeclareLaunchArgument(
            'node_prefix',
            default_value=[EnvironmentVariable('USER'), '_'],
            description='prefix for node name'
      ),
      Node(
            package='turtle_tf2_py',
            executable='fixed_frame_tf2_broadcaster',
            name=[LaunchConfiguration('node_prefix'), 'fixed_broadcaster'],
      ),
   ])" > fixed_broadcaster.launch.py

# modify setup.py (path is ${work_space}/src/launch_tutorial) according to https://docs.ros.org/en/galactic/Tutorials/Launch-Files/Using-ROS2-Launch-For-Large-Projects.html
# do not forget to import os and glob.glob packages
cd ${work_space}
colcon build
. install/setup.bash

