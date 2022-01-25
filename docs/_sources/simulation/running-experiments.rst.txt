Running experiments
===================

This documentation section shows how to run the experiments that are described in the thesis of this project.

Robot planner 1: Simple MoveIt planning
---------------------------------------

.. code-block:: bash

  roslaunch main program1.launch
  rosrun kuka_barrett robot_planner1


Robot Planner 2: Simulation layout and reachability experiments
---------------------------------------------------------------

To run the experiment 2a:
.. code-block:: bash

  roslaunch main program1.launch
  rosrun kuka_barrett robot_planner2a

and to run the experiment 2b run the following commands in different terminals
.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner2b


Robot Planner 3: Trajectory planning
------------------------------------

3a: Circular trajectories in task space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3a

3b: Line segment trajectories in task space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: ../images/3b_line_seg.png

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3b

3c: Cubic Spline trajectories in task space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3c

3d: B-Spline trajectories in task space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3d

3e: Polynomial trajectories in joint space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3e

3f: Trajectories in joint space with trapezoidal velocity profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3f

3g: Trajectories in joint space with s-curve velocity profile
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3g

3h: Helical trajectories in task space
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

  roslaunch main program3.launch
  rosrun kuka_barrett robot_planner3h


Robot Planner 4: Simple cube pick-and-place experiment
------------------------------------------------------

.. code-block:: bash

  roslaunch main program2.launch
  rosrun kuka_barrett robot_planner4


Robot Planner 5: Visual servoing
--------------------------------

To run the robot planner of this experiment first run the following commands in different terminals

.. code-block:: bash

  roslaunch main program2.launch
  rosrun kuka_barrett robot_planner5

to start the stereoscopic vision run the following launch file. Caution, the following launch files might be CPU and GPU intensive

.. code-block:: bash

  roslaunch vision stereo.launch

to run the visual servoing services run the following

.. code-block:: bash

  roslaunch vision visual_servo.launch


Robot Planner 6: RCM alignment error in insertion and retraction
----------------------------------------------------------------

.. image:: ../images/rcm-collision.png

To run the experiment run the folloei g commands in different terminals

.. code-block:: bash

  roslaunch main program2.launch
  rosrun kuka_barrett robot_planner6

To run the node that calculates the fulcrum error in real time, run the following command

.. code-block:: bash

  rosrun taskspace fulcrum_state_node

to plot the fulcrum error run the following

.. code-block:: bash

  rosrun rqt_plot rqt_plot /fulcrum/error

to inspect the nodes and topics of this experiment run the following ROS package

.. code-block:: bash

  rosrun rqt_graph rqt_graph


End-to-end simulation
------------------------

.. raw:: html
   
  <iframe width="690" height="400" src="https://www.youtube.com/embed/lfV1vdHf7bk" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

In separate terminals run the following commands

.. code-block:: bash

  roslaunch main program3_e2e.launch
  rosrun vision visual_servo.py
  rosrun kuka_barrett robot_planner7.py

or if you want more control over what to run, run the folloeing commands

.. code-block:: bash

  roslaunch main program3.launch
  roslaunch kuka_barrett action_servers.launch
  rosrun vision visual_servo.py
  rosrun taskspace fulcrum_state_node
  rosrun smach_viewer smach_viewer.py
  rosrun rqt_plot rqt_plot /fulcrum/error
  rosrun kuka_barrett robot_planner7.py



Spawn Surgical tool URDF
------------------------

.. code-block:: bash

  rosrun gazebo_ros spawn_model -file $(rospack find surgical_tools_description)/urdf/surgical_tool.urdf -urdf -x 1.0 -y 1.0 -z 1.2 -model surgical_tool_test