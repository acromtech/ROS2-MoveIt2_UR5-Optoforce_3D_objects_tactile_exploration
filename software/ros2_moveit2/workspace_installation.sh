#!/bin/bash

mkdir -p ~/ws_moveit_new2/src
cd ~/ws_moveit_new2/src

git clone https://github.com/ros-planning/moveit2.git -b iron
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git  -b iron
git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git  -b iron
git clone https://github.com/jkaniuka/optoforce_ros2.git
git clone https://github.com/ros-planning/moveit2_tutorials.git

pip3 install optoforce

vcs import < moveit2_tutorials/moveit2_tutorials.repos

sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/ws_moveit_new2

sed -i 's/moveit::core::JumpThreshold::relative(props.get<double>("jump_threshold")), is_valid,
/moveit::core::JumpThreshold(props.get<double>("jump_threshold")), is_valid,/gI' ~/ws_moveit_new2/src/moveit_task_constructor/core/src/solvers/cartesian_path.cpp
sed -i 's/const auto jump_thresh = moveit::core::JumpThreshold::disabled();/const moveit::core::JumpThreshold jump_thresh;/gI' ~/ws_moveit_new2/src/moveit2_tutorials/doc/how_to_guides/kinematics_cost_function/src/kinematics_cost_function_tutorial.cpp
sed -i 's/errorCodeToString/error_code_to_string/gI' ~/ws_moveit_new2/src/moveit2_tutorials/doc/how_to_guides/parallel_planning/src/parallel_planning_main.cpp
sed -i 's/pathLength/path_length/gI' ~/ws_moveit_new2/src/moveit2_tutorials/doc/how_to_guides/parallel_planning/src/parallel_planning_main.cpp

rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --mixin release --executor sequential

source ~/ws_moveit_new2/install/setup.bash

echo 'source ~/ws_moveit_new2/install/setup.bash' >> ~/.bashrc


ros2 launch optoforce_wrapper optoforce_test.launch.py
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5  use_fake_hardware:=true
