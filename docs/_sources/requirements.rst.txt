Requirements
============

.. highlight:: console

* This project is developed and tested in Ubuntu 18.04 (LTS) with ROS Melodic installed. ROS melodic (full installation) https://wiki.ros.org/melodic/Installation/Ubuntu
* Also the following packages must be installed:

  * Moveit

  .. code-block:: bash

    $ sudo apt-get install ros-melodic-moveit

  * Moveit visual tools

  .. code-block:: bash

    $ sudo apt-get install ros-melodic-moveit-visual-tools
    
  * Moveit Grasps (As of 14/3/2020 this package is not available and must be installed from source see `instructions in official repository <https://github.com/ros-planning/moveit_grasps#install-from-source>`_

  .. code-block:: bash
  
    $ sudo apt-get install ros-melodic-moveit-grasps

  * Joint Trajectory controller package

  .. code-block:: bash

    $ sudo apt-get install ros-kinetic-joint-trajectory-controller

  * ROS Controllers

  .. code-block:: bash

    $ sudo apt-get install ros-melodic-ros-controllers

  * Other dependencies

  .. code-block:: bash

  $ sudo apt-get install ros-melodic-cv-camera  

  * Catkin tools (optional)
  
  .. code-block:: bash

    $ sudo apt-get install ros-melodic-catkin python-catkin-tools  
