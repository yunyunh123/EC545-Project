<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->
<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

![product-screenshot](https://github.com/yunyunh123/EC545-Project/blob/main/Sources/Platooning.png)

This is our final project for EC545 Cyberphysical Systems at Boston University. We developed an autonomous vehicle platooning system leveraging ROS on the AgileX Limo Robots. In the current implementation, one manually controlled super-leader provides direction to a series of autonomous follower robots. Modifying the source code for the follower nodes allows for an unlimited number of additional followers to be added to the platoon. (ROS communications limitations and error propagation impose a practical limit on the number of followers that can be added to the platoon)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started
### Prerequisites
  - [ROS Humble](https://docs.ros.org/en/humble/index.html)
  - [Limo_ros](https://github.com/agilexrobotics/limo_ros)
  - [YDLidar_ros](https://github.com/YDLIDAR/ydlidar_ros_driver)
  - [YDLidar SDK](https://github.com/YDLIDAR/YDLidar-SDK)
<p align="right">(<a href="#readme-top">back to top</a>)</p>
<!-- USAGE EXAMPLES -->

## Usage

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
    
6) In catkin_ws/src/limo_controller/scripts/limo_node.py, change LIMO_ID to the correct id value (0 to n, where n+1 is the number of limos)

7) In catkin_ws/src/ydlidar_ros/launch/limo_lidar.launch, change the node name to "ydlidar_node_<id>" and the id parameter to "<id>". 

### Run code
1) Get LiDAR data
    * roslaunch ydlidar_ros limo_lidar
2) In catkin_ws/src/limo_controller/scripts:
    * Run python3 limo_node.py

### Potential errors and solutions
1) catkin_make
    * Error: CMake Error at CMakeLists.txt: Parse error. Expected a command name, got unquoted argument.
    * Soln: Remove catkin_ws/src/CMakeLists.txt

### Debug Tools
1) rostopic list
2) rostopic echo /[topic]
     - rosout topic dumps all data logged in nodes (rospy.loginfo())
4) rostopic pub \<topic-name\> \<topic-type\> \[data...\]

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Noah Cherry - ncherry@bu.edu
Julia Hua   - jhua2@bu.edu

<p align="right">(<a href="#readme-top">back to top</a>)</p>
