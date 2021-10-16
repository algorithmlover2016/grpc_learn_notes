# run in macos not container
xhost +localhost
docker run --ipc=host -itd -e DISPLAY=host.docker.internal:0 --privileged --restart=always --name "tutorial_ros2_ubuntu_2004_cudagl_1142_cp38_with_dis" nvidia/cudagl:11.4.2-devel-ubuntu20.04 /bin/bash

# run in docker container
locale # check for UTF-8
apt update && apt install locales
locale-gen en_US en_US.UTF-8
export LANG=en_US.UTF-8
locale # verify settings

# load and install basic software
apt update && apt install -y curl gnupg lsb-release
# Add the ROS 2 apt repository
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

# add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# install pip3:
apt-get install -y python3-distutils
mkdir -p ~/work_root/python_env && cd ~/work_root/python_env && curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py && cd -
# optional operation
apt install -y software-properties-common
apt install -y clang
apt install -y vim
echo "syntax enable
set hlsearch
set nu
set cindent
set shiftwidth=4
set softtabstop=4
set tabstop=4
set autoindent
set foldmethod=indent" > ~/.vimrc
export CC=clang
export CXX=clang++

cd ~/work_root
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

mkdir -p ~/work_root/ros2_galactic/src && cd ~/work_root/ros2_galactic
wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos && vcs import src < ros2.repos

rosdep init
rosdep update && rosdep install --from-paths src --ignore-src --rosdistro galactic -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

colcon build --symlink-install --cmake-force-configure
# build print stderror package
# foonathan_memory_vendor google_benchmark_vendor iceoryx_posh mimick_vendor qt_gui_cpp rmw_connextdds_common ros1_bridge rti_connext_dds_cmake_module

# set env for ros2 after build successfully
echo "export LANG=en_US.UTF-8
export CC=clang
export CXX=clang++
# if [ -f /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash ]; then
#   echo "source rti_connect_dds"
#     source /opt/rti.com/rti_connext_dds-5.3.1/setenv_ros2rti.bash > /dev/null
# fi

# set for ros2
if [ -f ~/work_root/ros2_galactic/install/local_setup.bash  ]; then
    source ~/work_root/ros2_galactic/install/setup.bash
fi
if [ -f /usr/share/colcon_cd/function/colcon_cd.sh ]; then
    source /usr/share/colcon_cd/function/colcon_cd.sh > /dev/null
    export _colcon_cd_root=~/work_root/ros2_galactic/install
    export ROS_DOMAIN_ID=0
fi" > ~/.bash_aliases

source ~/.bashrc
