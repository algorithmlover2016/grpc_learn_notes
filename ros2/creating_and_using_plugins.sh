export work_root="~/work_root"
export workspace="${work_root}/creating_and_using_plugins"
mkdir -p "${workspace}/src" && cd "${workspace}/src" 

# install pluginlib
sudo apt-get install ros-galactic-pluginlib

# create base class
cd "${workspace}/src" 
ros2 pkg create --build-type ament_cmake polygon_base --dependencies pluginlib --node-name area_node
# modify package.xml CMakeList.txt and include/polygon_base/regular_polygon.hpp

cd "${workspace}/src" 
ros2 pkg create --build-type ament_cmake polygon_plugins --dependencies polygon_base pluginlib --library-name polygon_plugins
