# create workspace:
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/ros/ros_tutorials.git -b galactic-devel
cd ../ # go to work_space
rosdep install -i --from-path src --rosdistro galactic -y
colcon build # --packages-up-to --symlink-install
# source your overlay
. install/local_setup.bash
# modify some changes
ros2 run turtlesim turtlesim_node # at ~/dev_ws which is the location that colon build run

# create packages:
# uses ament as its build system and colcon as its build tool. 
# CMake
cd ~/dev_ws/src
# ros2 pkg create --build-type ament_cmake <package_name>
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
cd ~/dev_ws
colcon build --packages-select my_package
. install/local_setup.bash
ros2 run my_package my_node

# python
cd ~/dev_ws/src
# ros2 pkg create --build-type ament_python <package_name>
ros2 pkg create --build-type ament_python --node-name my_node my_package_py
colcon build --packages-select my_package_py
. install/local_setup.bash # after run colcon build, we must run this command to add packages
ros2 run my_package_py my_node
