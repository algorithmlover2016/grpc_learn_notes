# install rqt
apt update && apt install -y ~nros-galactic-rqt*

# install turtlesim
apt update && apt install -y ros-galactic-turtlesim

# Understanding ROS 2 nodes
ros2 pkg executables turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key

# list command
ros2 node list
ros2 topic list
ros2 service list
ros2 action list

# remapping turtlesim_node for turtle_teleop_key
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel

# remapping turtlesim_node with different name
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle

# list info
ros2 node info /my_turtle

ros2 topic list -t

# ros2 topic echo <topic_name>
ros2 topic echo /turtle1/cmd_vel
ros2 topic echo /turtle1/pose

# ros2 topic info <topic_name>
ros2 topic info /turtle1/cmd_vel


# ros2 interface show <msg_type>
ros2 interface show geometry_msgs/msg/Twist
# ros2 topic pub <topic_name> <msg_type> '<args>'
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# ros2 topic hz <topic_name>
ros2 topic hz /turtle1/pose

rqt
rqt_graph


