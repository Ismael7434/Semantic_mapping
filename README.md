# Semantic_mapping
2D Semantic Mapping project for indoor mobile robots.  The system integrates Faster R-CNN for object detection, point cloud processing for semantic map construction,  and navigation modules including localization, trajectory planning, and obstacle avoidance.

This ROS project provides a set of nodes, launch files, and configuration files to control a mobile robot. It includes functionality for sensor integration, motion planning, and robot visualization using RViz. The system is designed to control a robot using a differential drive model.
Table of Contents

    Introduction
    Components
    Installation
    Usage
    Contact

Introduction

    This project aims to establish collaborative semantic mapping with mobile robots using ROS. It includes nodes for sensor data processing (e.g., LIDAR, odometry), motion control, and visualization in RViz. The system supports launching the robot’s main control system through launch files and provides various parameters to adjust robot behavior such as speed, sensor configurations, and path planning.

Hardware
    
    Ubuntu 20.04.6 LTS
    VIDIA Corporation GP102 [GeForce GTX 1080 Ti] (rev a1)
    NVIDIA (R) Cuda compiler driver, release 11.2

Components

The main components of this repository are:

    ROS Nodes:
        sensor_node: Reads and processes sensor data from LIDAR and other sensors.
        motion_control_node: Handles the robot's movement based on odometry and commands from the control system.
        path_planner_node: Plans a path for the robot based on sensor data and maps.
        teleop_node: Allows manual control of the robot through keyboard inputs.

    Packages:
        robot_control: Contains the main control logic for the robot, including movement and path planning.
        robot_sensors: Integrates sensor data and provides information on the robot's surroundings.
        robot_visualization: Contains RViz configurations and visualization markers for robot models.

    Launch Files:
        start_robot.launch: Launches the entire robot system including sensors, control, and path planning.
        start_teleop.launch: Launches teleoperation node to control the robot manually.
        start_rviz.launch: Starts RViz with pre-configured settings for robot visualization.

    Messages/Services:
        Custom messages for robot status and sensor data.
        Custom service for controlling robot movement.

    Dependencies:
        roscpp, sensor_msgs, geometry_msgs, move_base, tf, laser_geometry

Installation

Follow these steps to install and set up the project on your local machine:
    
    Ensure you have ROS installed. Follow the installation guide at ROS Installation.

    Clone the repository

        Install ROS dependencies:

    Install the required dependencies using rosdep:

        rosdep install --from-paths src --ignore-src -r -y


Build the workspace:

Build the workspace using catkin_make:

    catkin_make

Source the workspace:

Source the workspace setup file:

    source devel/setup.bash

Usage :

Once installed, you can use the system to establish semantic mapping:

To start the robot's complete system, including localisation, navigation and rviz visualisation, use :

    roslaunch mir_gazebo mir_autonomous.launch

This will launch all necessary nodes and start the robot's control system.

To launch the semantic mapping node, run in another cmd:
    
 rosrun semmapping mapping _camera_info:=/camera/rgb/camera_info 2> >(grep -v TF_REPEATED_DATA buffer_core)


Navigate the robot in the environment :

    If you want to control the robot manually using a RVIZ, use the "2D Nav Goal" button in RVIZ to give successive trajectory points.

If you want to control the robot autonomously :

   roslaunch trajectory_follower trajectory_follower.launch


To create an autonomous path for the robot, use :

    rostopic pub /path_ready std_msgs/Empty -1

To navigate the environment using this path, use :


To launch the semantic map update algorithm, use :



For questions or support, feel free to contact the project maintainers:

    Email: diakiteismael7434@gmail.com

    Tel:0745441588

