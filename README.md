-------- USER GUIDE ----------

build the packages first 
(only if you're making any new packages or changing any file content and want to access the new content)

use colcon build at root level (WoodFISH_sim) directory level
or if you just need to build these packages -> controller : /usr/bin/colcon build --packages-select controller

then source the newly built pkgs

source install/setup.bash

to launch the file :

ros2 launch robot_description gazebo_model.launch.py

to make the bot move around in the pool :

in another terminal

ros2 run controller teleop_auv_keyboard

a layout will open in the terminal screen

q -> ascent
w -> forward
e -> descent

a-> left
s-> stop
d -> right

x -> backward
