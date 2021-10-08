# reference to https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html and
# https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Service-And-Client.html
export work_root="~/work_root"
export work_space="${work_root}/service_and_client/"
mkdir -p ${work_space}/src && cd ${work_space}/src

# cpp version
ros2 pkg create --build-type ament_cmake cpp_srvcli --dependencies rclcpp example_interfaces
cd ${work_space}/src/cpp_srvcli/src
wget -O add_two_ints_client.cpp https://github.com/ros2/examples/blob/galactic/rclcpp/services/minimal_client/main.cpp
wget -O add_two_ints_server.cpp https://github.com/ros2/examples/blob/galactic/rclcpp/services/minimal_service/main.cpp

cd ${work_space}/src/cpp_srvcli/
# modify CMakeLists.txt to add executable and install target

cd ${work_space}/
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select cpp_srvcli

# for test
# in one terminal
cd ${work_space}/
. install/setup.sh
ros2 run cpp_srvcli client 50 49
ros2 run cpp_srvcli client_github

# in another terminal
cd ${work_space}/
. install/setup.sh
ros2 run cpp_srvcli server_github
ros2 run cpp_srvcli server

# py version
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
wget -O service_member_function.py https://github.com/ros2/examples/blob/galactic/rclpy/services/minimal_service/examples_rclpy_minimal_service/service_member_function.py
wget -O client_member_function.py https://github.com/ros2/examples/blob/galactic/rclpy/services/minimal_client/examples_rclpy_minimal_client/client.py

# 
cd ${work_space}/
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select py_srvcli

# for test
# in one terminal
cd ${work_space}/ && . install/setup.sh
ros2 run cpp_srvcli service

# in another terminal
cd ${work_space}/ && . install/setup.sh
ros2 run cpp_srvcli client_github
ros2 run cpp_srvcli client 289 897
