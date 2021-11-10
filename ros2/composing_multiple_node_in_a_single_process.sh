# Run-time composition using ROS services
# Firstly, run the following command in one shell terminal
ros2 run rclcpp_components component_container
# Secondly, in one shell terminal
ros2 component list
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
ros2 component list
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
ros2 component list
# Unloading components
ros2 component unload /ComponentManager 1 2 # 1, 2 is the component id

# Compile-time composition using ROS services
# https://github.com/ros2/demos/blob/master/composition/src/manual_composition.cpp
ros2 run composition manual_composition

# Run-time composition using dlopen
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

# Composition using launch actions
ros2 launch composition composition_demo.launch.py

# Remapping container name and namespace
# Firstly, run the following command in one shell terminal
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

# Secondly, run the following commands in another shell terminal
ros2 component load /ns/MyContainer composition composition::Listener

# Remap component names and namespaces
# Firstly, run the following command in one shell terminal
ros2 run rclcpp_components component_container

# Secondly, run the following commands in another shell terminal
# Remap node name
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
# Remap namespace
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
# Remap both
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
Notice:
# run the following command can output the following results:
ros2 component list
# the output is:
/ComponentManager
  1  /talker2
  2  /ns2/talker3
  3  /ns2/talker3
/ns/MyContainer
  1  /listener
  2  /talker
  3  /talker
