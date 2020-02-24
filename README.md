Surgery Robotics with KUKA LBR iiwa and Barrett
================================================

## Third-party software, assets & references

- LBR iiwa [https://github.com/ros-industrial/kuka_experimental]()
- KUKA LBR iiwa and Barrett Hand [https://github.com/karahbit/Gazebo-Manual-Pick-and-Place-iiwa14]()

## Instructions

- ROS melodic (full installation) [https://wiki.ros.org/melodic/Installation/Ubuntu]()
- Moveit `sudo apt-get install ros-melodic-moveit`
- Joint TRajectory controller package `sudo apt-get install ros-kinetic-joint-trajectory-controller`
- In the root directory of the project run:
```
catkin_make
source devel/setup.bash # or setup.sh, setup.zsh
```
- Run gazebo `roslaunch roslaunch kuka_barrett_control kuka_barrett_control.launch`
- Run moveit in RViz `roslaunch iiwa_moveit_config demo.launch`