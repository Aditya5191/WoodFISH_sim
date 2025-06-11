-------- USER GUIDE ----------

build the packages first 
(only if you're making any new packages or changing any file content and want to access the new content)

use colcon build at root level (WoodFISH_sim) directory level
or if you just need to build these packages -> controller : /usr/bin/colcon build --packages-select controller

then source the newly built pkgs

source install/setup.bash

to launch the file :

ros2 launch robot_description gazebo_model.launch.py

to make use of the thrusters :

in another terminal

ros2 topic pub /cmd_tsup std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 30.0, 30.0]" -- example

first check in ros2 topic list and gz topic list if thruster nodes exist and accordingly change value as needed.
