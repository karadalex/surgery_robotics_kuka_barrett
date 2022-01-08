Installation Instructions
=========================

Setup
-----

Clone this repository with **--recurse-submodules** flag to download submodule repositories

.. code-block:: bash

  catkin_make
  source devel/setup.bash # or setup.sh, setup.zsh


Optional Instructions
---------------------

Import surgical tools to Gazebo database
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Copy the folder **surgical_tool** which is inside **src/kuka_barrett_gazebo/objects** and paste it in **/home/<YOUR_USERNAME>/.gazebo/models**. DO the same for **mounting_dock**

Install CoppeliaSim (VREP) with ROS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Install VREP
2. Make sure to have installed the necessary libraries. e.g. for **tf2_sensor_msgs** run `sudo apt-get install ros-melodic-tf2-sensor-msgs`
3. Create a new workspace 

.. code-block:: bash

  source /opt/ros/melodic/setup.bash
  mkdir -p ~/vrep_ws/src
  cd ~/vrep_ws/
  catkin_make

4. Install xsltproc ``sudo apt-get install xsltproc``
5. Download plugin source code and build it

.. code-block:: bash

  cd src
  git clone --recursive https://github.com/CoppeliaRobotics/simExtROSInterface.git sim_ros_interface
  source devel/setup.bash
  export COPPELIASIM_ROOT_DIR=/home/user/path-to-your-vrep-installation-folder
  cd ~/vrep_ws/
  catkin_make
  cd devel/lib
  cp libsimExtROSInterface.so $COPPELIASIM_ROOT_DIR

6. Run VREP (after you have sourced your ROS workspace environment) ``$COPPELIASIM_ROOT_DIR/coppeliaSim.sh``. 
While running make sure to check in the terminal that the plugin is loaded successfully.
