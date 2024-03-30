# Turtlebot Map Navigation and Parts Search

![image](https://github.com/Shyam-pi/Aruco-based-Map-Navigation/assets/57116285/ef8bd364-fb6c-45c5-8fa2-f98df9376c24)

## Overview
The project focuses on creating a ROS package to perform various tasks like detecting aruco markers, and using logical cameras' outputs to find different parts in a map, for a Turtlebot.

## Objectives
- Create a ROS package to perform tasks like map creation, ArUco marker detection, logical camera usage, etc.
- Develop ROS nodes, publishers, and subscribers.
- Utilize parameters and write an action client to navigate through poses.

## Features
- **Simulation Environment:** Start the simulation environment with the provided launch file.
- **ArUco Marker Detection:** Read ArUco markers and retrieve parameters associated with them.
- **Logical Camera Usage:** Detect parts using logical cameras in the environment.
- **Pose Navigation:** Write an action client to navigate the Turtlebot through poses in a specific order.
- **Documentation:** Ensure all code, methods, and attributes are properly documented using Doxygen.

# Running the Code
1. Paste the package under the src folder in the workspace, the path is final_ws/src if step 1 is followed.
2. Ensure that you "export TURTLEBOT3_MODEL=waffle ".
3. Make sure you have cd into the root of the workspace and the package is sourced as "source install/setup.bash".
4. Please run "ros2 launch final_project final_project.launch.py".
5. After rviz has opened up with the gazebo world spawned with the robot and map ,Please run the following command "ros2 launch group7_final test.launch.py".
6. The robot will start navigation and can been seen in gazebo and rviz

#  Output
The video recording of the same package can be found at https://drive.google.com/file/d/1oFRuKR9_YJDQGJu6jlrGzA5-5If8gBDy/view?usp=sharing.
