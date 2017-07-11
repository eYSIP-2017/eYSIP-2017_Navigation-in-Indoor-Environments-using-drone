## Directory Structure of the package

This is a ROS Package and the basic layout is the same as any ROS package.
* action
    * Holds all the actions created
    * move.action - action for movement along a trajectory
* include
    * holds any libraries that need to be included
* launch
    * holds all the launch files
    * aruco_mapping.launch - starts the aruco mapping functionality 
    * for_real_map.launch - motion planning using moveit for real world
    * for_simulated_map.launch - motion planning using moveit for simulated environment
    * localisation.launch - start functionality for using EKF (untested)
    * simulator.launch - launch simulator
    * single_aruco.launch - following a single aruco marker
* msg
    * holds custom ros messaged created
    * pid_error.msg - holds array data used to store pid errors
* scripts
    * holds python scripts related to the functionality of the package
    * ardrone teleop key.py - Handle Control of drone using keyboard
    * pose.py - Class for pose storage
    * localisation.py - Handle usage of EKF
    * kalman filter.py - Implimenation of Extended Kalman Filter
    * move to waypoint.py - Get drone to follow generated Trajectory
    * follow trajectory.py - Extract, generate and send trajectory for excecution
    * pid.py - Generic PID implimentation
    * transform handler.py - Generate transform between drone’s odom and aruco’s world

The other folders added are as follows
* aruco_models
    * holds gazebo aruco_models needed for simulation
* camera_calibration_files
    * holds the camera calibration files for ardrone
    * it is recomended that you create these files on your own
* Documents
    * holds all the documents prepared related to the project
* install_scripts
    * holds bash scripts for installing packages automatically
    * complete_install.sh - installs everything needed for the use of this project
* worlds
    * holds the test worlds that we had created
    * small_world.sdf - small test world created
    * small_world_with_aruco.sdf - small test world created with arucos loaded

#### Difference in other branches

* **whycon**
    * whycon_formation_data
        * holds files and scripts used for training formation based identification of whycon
        * *.txt files are data collected
        * *.p files are pretrained models
        * store_formation_data.py - data collection script
        * extract_results_from_whycon.py - using the data to train and draw inferences
    * scripts
        * detect_whycon.py - using pretrained objects to find the the current formation group