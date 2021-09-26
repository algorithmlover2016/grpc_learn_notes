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

# services
ros2 service list
ros2 service list -t # same to # ros2 service list --show-types

# ros2 service type <service_name>
ros2 service type /clear

# ros2 service find <type_name> # find all the services of a specific type
ros2 service find std_srvs/srv/Empty

# ros2 interface show <type_name>
ros2 interface show std_srvs/srv/Empty
# ros2 service call
# ros2 service call <service_name> <service_type> <arguments>
ros2 service call /clear std_srvs/srv/Empty
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
ros2 service call /spawn turtlesim/srv/Spawn "{'x': 2, 'y': 2, 'theta': 0.2, 'name': ''}"
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r': 255, 'g': 0, 'b': 0, 'width': 5, 'off': 0}"
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{'r':255, 'g':128, 'b':255, 'width':5, 'off':0}"

## understanding ros2 parameters
# ros2 param list
ros2 param list
ros2 param list --param-type

# ros2 param get <node_name> <parameter_name>
ros2 param get /turtlesim background_r

# ros2 param set <node_name> <parameter_name> <value>
ros2 param set /turtlesim background_r 150

# ros2 param dump <node_name>
ros2 param dump /turtlesim

# ros2 param load <node_name> <parameter_file>
ros2 param load /turtlesim ./turtlesim.yaml

# 7 Load parameter file on node startup
# ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
ros2 run turtlesim turtlesim_node --ros-args --params-file ./turtlesim.yaml

## Understanding ROS 2 actions
# reference to https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Actions.html

# ros2 node info <node_name>
ros2 node info /turtlesim
ros2 node info /teleop_turtle

# ros2 action list
ros2 action list
ros2 action list -t

# ros2 action info <action_name>
ros2 action info /turtle1/rotate_absolute

# ros2 interface show <action_type>
ros2 interface show turtlesim/action/RotateAbsolute

# ros2 action send_goal <action_name> <action_type> <values>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{"theta": 6}"
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{'theta': 6}"
# with feedback
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback
