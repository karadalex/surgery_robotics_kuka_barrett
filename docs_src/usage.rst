Running the simulation
======================

Usage
-----

List of available programs (launch files) to run:

1. Testing some simple trajectories with the tool already attached to robot.
  .. code-block:: bash

    $ roslaunch main program1.launch
    $ rosrun kuka_barrett robot_planner1

2. `main program2.launch` : Simple pick and place example program
3. (Discontinued) ``roslaunch kuka_barrett_control computed_torque_controller_test.launch``

To run program 1, run in a sourced terminal ``roslaunch main program1.launch`` This will run a gazebo simulator with robot, tables and tools, RViz with Moveit plugins and camera 
views and an OpenCV Node.


Convenience scripts
-------------------

- ``rosrun main iiwa_home.sh`` move iiwa arm to home position
- ``rosrun main barrett_home.sh`` move Barrett hand to home position