# reference to https://docs.ros.org/en/galactic/Tutorials/Custom-ROS2-Interfaces.html and
# https://docs.ros.org/en/galactic/Tutorials/Single-Package-Define-And-Use-Interface.html
export work_root="~/work_root"
export work_space="${work_root}/custom_interface_msg_srv/"
mkdir -p ${work_space}/src && cd ${work_space}/src

# custom interfaces
ros2 pkg create --build-type ament_cmake tutorial_interfaces
cd ${work_space}/src/tutorial_interfaces
mkdir msg && mkdir srv
cd ${work_space}/src/tutorial_interfaces/msg
echo "int64 num" > Num.msg
cd ${work_space}/src/tutorial_interfaces/srv
echo "int64 a
int64 b
int64 c
---
int64 sum" > AddThreeInts.srv

# modify the CMakeLists.txt and package.xml to build the package

# rosdep install -i --from-path src --rosdistro galactic -y # can be omit
cd ${work_space}
colcon build --packages-select tutorial_interfaces
. install/setup.bash

# test interface
cd ${work_space}
ros2 interface show tutorial_interfaces/msg/Num
ros2 interface show tutorial_interfaces/srv/AddThreeInts

# then use the interface to test publisher/subscriber and server/client

# Expanding on ROS 2 interfaces
cd ${work_space}/src
ros2 pkg create --build-type ament_cmake more_interfaces
mkdir more_interfaces/msg
# add necessary files at right location, according to 

cd ${work_space}
colcon build --packages-up-to more_interfaces
. install/local_setup.bash
ros2 run more_interfaces publish_address_book
# learn about more_interfaces publish_address_book_array to learn how to use an existing interface definition
