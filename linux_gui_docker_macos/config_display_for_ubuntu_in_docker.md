brew upgrade
brew install socat
brew install xquartz
open -a Xquartz
# modify the security in preferce to allow connections from network clients
if [ "$(lsof -nP -i4TCP:6000 | grep -c LISTEN)" -eq 0 ]; then
  socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\"
fi
# choice one
# reference to
# https://rocky.dev/running-linux-gui-apps-in-docker-on-mac and
# https://cntnr.io/running-guis-with-docker-on-mac-os-x-a14df6a76efc and
# https://techsparx.com/software-development/docker/display-x11-apps.html and
# https://kartoza.com/en/blog/how-to-run-a-linux-gui-application-on-osx-using-docker/
IP_ADDRESS=$(ifconfig en0 | grep "inet " | cut -d " " -f 2)
echo $IP_ADDRESS
# docker create -it -e DISPLAY=$IP_ADDRESS:0 -v $CODE_DIR:/home/dev/ $IMAGE_ID bash
# docker run --ipc=host -it -e DISPLAY=10.16.96.20:0 --privileged --name "tutorial_ros2_ubuntu_2004_cp38_with_dis" demo_ros2_ubuntu_2004_cp38_build_galactic /bin/bash
docker run --ipc=host -it -e DISPLAY=$IP_ADDRESS:0 --privileged --name "tutorial_ros2_ubuntu_2004_cp38_with_dis" demo_ros2_ubuntu_2004_cp38_build_galactic /bin/bash

# choice two reference to https://shaoguangleo.github.io/2018/01/21/docker-run-gui-on-macosx/
# in macos:
xhost +localhost
# start docker
docker run --ipc=host -it -e DISPLAY=host.docker.internal:0 --privileged --name "tutorial_ros2_ubuntu_2004_cp38_with_dis" demo_ros2_ubuntu_2004_cp38_build_galactic /bin/bash
