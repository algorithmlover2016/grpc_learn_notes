# when install with root:
## set locale
locale  # check for UTF-8

apt update && apt install locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

## Add the ROS 2 apt repository
apt update && apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools and ROS tools
# install pip3:
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py
# python3-pip \
apt update && apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# install with binary package
https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html
# required packages in python
 catkin_pkg
 empy

# solve build error:
# 8 packages had stderr output:
    # foonathan_memory_vendor google_benchmark_vendor iceoryx_posh mimick_vendor qt_gui_cpp
    # rmw_connextdds_common ros1_bridge rti_connext_dds_cmake_module

    # qt_gui_cpp
    apt install clang
    export CC=clang
    export CXX=clang++

    # rmw_connextdds_common
    # reference to https://docs.ros.org/en/galactic/Installation/DDS-Implementations.html
    apt install -q -y \
    rti-connext-dds-5.3.1
    cd /opt/rti.com/rti_connext_dds-5.3.1/resource/scripts && source ./rtisetenv_x64Linux3gcc5.4.0.bash; cd -
    export CONNEXTDDS_DIR=${NDDSHOME}

    colcon build --symlink-install --cmake-force-configure

# ~/.bash_aliases
export LANG=en_US.UTF-8
# export CC=clang
# export CXX=clang++
# # if you want to install rti_connext_dds
# if [ -f /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash ]; then
#     source /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash > /dev/null
#     export CONNEXTDDS_DIR=${NDDSHOME}
# fi

if [ -f ~/work_root/ros2_galactic/install/local_setup.bash  ]; then
	source ~/work_root/ros2_galactic/install/local_setup.bash > /dev/null
    export _colcon_cd_root=~/work_root/ros2_galactic
	export ROS_DOMAIN_ID=0
fi

# update ros2
# reference to https://docs.ros.org/en/galactic/Installation/Maintaining-a-Source-Checkout.html#maintainingsource

# rmw implementations and change
# https://docs.ros.org/en/galactic/Installation/DDS-Implementations.html
# https://docs.ros.org/en/galactic/Guides/Working-with-multiple-RMW-implementations.html

rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"
