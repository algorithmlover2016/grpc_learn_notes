# reference to https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html and
# https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
# create workspace
export work_root="~/work_root"
mkdir -p ${work_root}/publisher_and_subscriber_topic/src && cd ${work_root}/publisher_and_subscriber_topic/src

# cpp version
# create package
ros2 pkg create --build-type ament_cmake cpp_pubsub

# get publisher cpp code
cd ${work_root}/publisher_and_subscriber_topic/src/cpp_pubsub/src
wget -O publisher_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_publisher/member_function.cpp
wget -O subscriber_member_function.cpp https://raw.githubusercontent.com/ros2/examples/master/rclcpp/topics/minimal_subscriber/member_function.cpp
# modify the cpp code if necessary
cd ${work_root}/publisher_and_subscriber_topic/src/cpp_pubsub/
# add dependencies for package.xml and CMakeLists.txt

# check and add dependency for ros2 if necessary
cd ${work_root}/publisher_and_subscriber_topic/
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select cpp_pubsub

source ./install/setup.bash # or . install/setup.bash

# run the executable in different terminal
source ./install/setup.bash && ros2 run cpp_pubsub listener
source ./install/setup.bash && ros2 run cpp_pubsub talker

# py version
ros2 pkg create --build-type ament_python py_pubsub
cd ${work_root}/publisher_and_subscriber_topic/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
wget https://raw.githubusercontent.com/ros2/examples/master/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
cd ${work_root}/publisher_and_subscriber_topic/src/py_pubsub/
# add dependency for package.xml and modify entrypoint in setup.py

# check and add dependency for ros2 if necessary
cd ${work_root}/publisher_and_subscriber_topic/
rosdep install -i --from-path src --rosdistro galactic -y
colcon build --packages-select py_pubsub
. install/setup.bash
ros2 run py_pubsub talker

# open in another terminal
cd ${work_root}/publisher_and_subscriber_topic/ && source install/setup.bash
ros2 run py_pubsub listener
