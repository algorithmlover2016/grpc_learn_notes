# reference to https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html and
# https://docs.ros.org/en/galactic/Tutorials/Using-Parameters-In-A-Class-Python.html
export work_root="~/work_root"
export workspace="${work_root}/using_parameters_in_class"
mkdir -p "${workspace}/src" && cd "${workspace}/src"

# cpp version
cd "${workspace}/src"
ros2 pkg create --build-type ament_cmake cpp_parameters --dependencies rclcpp
# modify the package.xml and CMakeList.txt and src/parameter_node.cpp

cd "${workspace}"
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select cpp_parameters
. install/setup.bash
ros2 run cpp_parameters parameter_node
# change the user self-defined parameters
# Change via the console
# show the params in running node
ros2 param list
# set value for specified parameter in a running node
ros2 param set /parameter_node my_parameter earth
# Change via a launch file
cd "${workspace}/src/cpp_parameters"
mkdir "launch" && cd "launch"
echo "from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])" > cpp_parameters_launch.py
# modify CMakeList.txt 
colcon build --packages-select cpp_parameters
. install/setup.bash
ros2 launch cpp_parameters cpp_parameters_launch.py

# py version
cd "${workspace}/src"
ros2 pkg create --build-type ament_python python_parameters --dependencies rclpy
# modify package.xml and setup.py and add py executable script

cd "${workspace}"
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select python_parameters
. install/setup.bash
ros2 run python_parameters param_talker
# change parameter
# from console when running the node
ros2 param list
ros2 param set /minimal_param_node my_parameter earth
# from lauch file
cd "${workspace}/src/python_parameters"
mkdir "launch" && cd "launch"
echo "from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='param_talker',
            name='custom_parameter_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])" > python_parameters_launch.py
cd "${workspace}/src/python_parameters"
# modify setup.py. Don't forget to import os and glob, and modify data_files
cd "${workspace}"
colcon build --packages-select python_parameters
. install/setup.bash
ros2 launch python_parameters python_parameters_launch.py
