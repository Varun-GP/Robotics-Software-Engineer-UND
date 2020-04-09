#!/bin/sh

# launch turtlebot_world.launch to deploy turtlebot environment
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch  world_file:=$(pwd)/../../src/map/Home.world " & 
sleep 5

# launch amcl_demo.launch for localization
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)/../../src/map/Home_map.yaml " &
sleep 5

# launch view_navigation for rviz
#xterm -e "cd $(pwd)/../..;
#source devel/setup.bash;
#roslaunch turtlebot_rviz_launchers view_navigation.launch" &

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch add_markers home_rviz_config.launch rviz_config_file:=$(pwd)/../rvizConfig/home.rviz" &
sleep 20 # keeping large to enable visualization

# launch add_markers node
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../config/marker_config.yaml;
rosrun add_markers add_markers_test " &
