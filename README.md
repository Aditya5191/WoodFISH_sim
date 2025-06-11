-------- USER GUIDE ----------

build the packages first 

use colcon build at root level (WoodFISH_sim) directory level
or if you just need to build these packages -> controller : /usr/bin/colcon build --packages-select controller

to launch the file :

ros2 launch robot_description gazebo_model.launch.py

to make use of the thrusters :

in another terminal

ros2 topic pub /cmd_tsup std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 30.0, 30.0]" -- example

first check in ros2 topic list and gz topic list if thruster nodes exist and accordingly change value as needed.
