# Limo Controller

## Setup
Run the following commands:
1) catkin_make  

2) Set up ROS: run either of the 2 commands
* source /opt/ros/<ROS_distribution>/setup.bash  
* C:\opt\ros\<ROS_distribution>\x64\setup.bat  

3) Set up project: run either of the 2 commands
* source ./devel/setup.bash  
* .\devel\setup.bat

4) On central controller: X.X.X.X and Y.Y.Y.Y are both central controller's IP address
* export ROS_MASTER_URI=http://X.X.X.X:11311 (or set as environment variable)
* export ROS_IP=Y.Y.Y.Y
* roscore

5) On limo nodes: X.X.X.X is central controller's IP address, and Y.Y.Y.Y is this limo node's IP address
* Ensure node is on the same local WiFi as central controller
* export ROS_MASTER_URI=http://X.X.X.X:11311
* export ROS_IP=Y.Y.Y.Y

## Run code
1) Get LiDAR data
* roslaunch ydlidar_ros X2L.launch
* rosrun ydlidar_ros ydlidar_client

## Potential errors and solutions
1) catkin_make
* Error: CMake Error at CMakeLists.txt: Parse error. Expected a command name, got unquoted argument.
* Soln: Remove catkin_ws/src/CMakeLists.txt


