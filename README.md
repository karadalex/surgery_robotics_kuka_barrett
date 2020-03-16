Surgery Robotics with KUKA LBR iiwa and Barrett
================================================

## Requirements

- This project is developed and tested in Ubuntu 18.04 (LTS) with ROS Melodic installed. ROS melodic (full installation) [https://wiki.ros.org/melodic/Installation/Ubuntu]()
- Also the following packages must be installed:
  - Moveit `sudo apt-get install ros-melodic-moveit`
  - Moveit visual tools `sudo apt-get install ros-melodic-moveit-visual-tools`
  - Moveit Grasps `sudo apt-get install ros-melodic-moveit-grasps` (As of 14/3/2020 this package is not available and must be installed from source see [instructions in official repository](https://github.com/ros-planning/moveit_grasps#install-from-source))
  - Joint Trajectory controller package `sudo apt-get install ros-kinetic-joint-trajectory-controller`
  - ROS Controllers `sudo apt-get install ros-melodic-ros-controllers`
  - Catkin tools (optional) `sudo apt-get install ros-melodic-catkin python-catkin-tools`

## Instructions

- Clone this repository with **--recurse-submodules** flag to download submodule repositories
```
catkin_make
source devel/setup.bash # or setup.sh, setup.zsh
```

### Import surgical tools to Gazebo database

Copy the folder **surgical_tool** which is inside **src/kuka_barrett_gazebo/objects** and paste it in **/home/<YOUR_USERNAME>/.gazebo/models**

### Install CoppeliaSim (VREP) with ROS (optional)

1. Install VREP
2. Make sure to have installed the necessary libraries. e.g. for **tf2_sensor_msgs** run `sudo apt-get install ros-melodic-tf2-sensor-msgs`
3. Create a new workspace 
```
source /opt/ros/melodic/setup.bash
mkdir -p ~/vrep_ws/src
cd ~/vrep_ws/
catkin_make
```
4. Install xsltproc `sudo apt-get install xsltproc`
5. Download plugin source code and build it
```
cd src
git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
source devel/setup.bash
export COPPELIASIM_ROOT_DIR=/home/user/path-to-your-vrep-installation-folder
cd ~/vrep_ws/
catkin_make
cd devel/lib
cp libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR
```
6. Run VREP (after you have sourced your ROS workspace environment) `$COPPELIASIM_ROOT_DIR/coppeliaSim.sh`. 
While running make sure to check in the terminal that the plugin is loaded successfully.


## Usage

- To run main program run in a sourced terminal 
```
roslaunch main program1.launch
```
This will run a gazebo simulator with robot, tables and tools, RViz with Moveit plugins and camera 
views and an OpenCV Node.

- To run the **Computed Torque Controller** test run 
```
roslaunch kuka_barrett_control computed_torque_controller_test.launch
```
which will run gazebo with EffortJointInterface, the ComputedTorqueControl spawned in Controller Manager 
and a rqt window with joint states plots.


## Troubleshooting

- _UnicodeDecodeError: 'ascii' codec can't decode byte 0xce in position 33: ordinal not in range(128)_ : Make sure the project path doesn't contain any greek characters
- _Eigen/Geometry: No such file or directory_: `sudo apt-get install libeigen3-dev` if that does not work either `sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen`

## References

See [REFERENCES.md](./REFERENCES.md)