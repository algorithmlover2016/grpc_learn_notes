export work_root="~/work_root"
export workspace="${work_root}/action_ws"
mkdir -p "${workspace}/src" && cd "${workspace}/src" 

ros2 pkg create action_tutorials_interfaces
cd action_tutorials_interfaces && mkdir action && cd action

echo "int32 order
---
int32[] sequence
---
int32[] partial_sequence" > ./Fibonacci.action

# modify package.xml and CMakeList.txt in "${workspace}/src/action_tutorials_interfaces"  to add build and dependency

cd "${workspace}" 
# Source our workspace
# On Windows: call install/setup.bat
. install/setup.bash
# Check that our action definition exists
ros2 interface show action_tutorials/action/Fibonacci
ros2 action info action_tutorials/action/Fibonacci

# cpp version
# write an action server and client, reference to https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Cpp-Action-Server-Client.html
cd "${workspace}/src"
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

# add some modification for action_server and action_client on related files(.h, .cpp and .xml, and CMakeList.txt)

cd "${workspace}"
colcon build
# run the server first
. install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_server

# run the client later
. install/setup.bash
ros2 run action_tutorials_cpp fibonacci_action_client

## py version, reference to https://docs.ros.org/en/galactic/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html and
# https://github.com/ros2/demos/tree/master/action_tutorials/action_tutorials_py
cd "${workspace}/src"
ros2 pkg create --dependencies action_tutorials_interfaces rclpy --build-type ament_python action_tutorials_py

cd "${workspace}/src/action_tutorials_py/action_tutorials_py"
# add fibonacci_action_server.py

# run the py
python3 fibonacci_action_server.py
# in another terminal test server.py
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
# add feedback test
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

cd "${workspace}/src/action_tutorials_py/action_tutorials_py"
# add fibonacci_action_client.py
# after run the server, we can test the client

cd "${workspace}/src/action_tutorials_py/"
# modify the setup.py, mainly add entry_points.'console_scripts' elements, about node_name of server and client

cd "${workspace}"
colcon build

# run the server first
. install/setup.bash
ros2 run action_tutorials_py fibonacci_action_server

# run the client in another terminal
. install/setup.bash
ros2 run action_tutorials_py fibonacci_action_client
ros2 run action_tutorials_py fibonacci_action_server
