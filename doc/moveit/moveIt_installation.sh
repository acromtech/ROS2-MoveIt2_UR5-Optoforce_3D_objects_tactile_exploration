#!/bin/bash

source /opt/ros/iron/setup.bash

sudo apt install python3-rosdep

sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade

sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

sudo apt install python3-vcstool

mkdir -p ~/ws_moveit/src

cd ~/ws_moveit/src
git clone https://github.com/ros-planning/moveit2_tutorials

vcs import < moveit2_tutorials/moveit2_tutorials.repos

sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/ws_moveit
colcon build --mixin release

source ~/ws_moveit/install/setup.bash

echo 'source ~/ws_moveit/install/setup.bash' >> ~/.bashrc

ros2 launch moveit2_tutorials demo.launch.py
