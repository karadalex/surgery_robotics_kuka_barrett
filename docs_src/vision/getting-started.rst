Getting Started
===============

Vision Launch file
------------------

After having launched the main simulation with Gazebo run the following launch file to start 
the stereo image processing nodes

.. code-block:: bash

  $ roslaunch vision vision.launch

To use real cameras edit the ``vision.launch`` file and uncomment the following section

.. code-block:: xml

  <node
      pkg="cv_camera"
      type="cv_camera_node"
      name="left" 
      args="$(arg cam1)" >
  </node>
  <node
      pkg="cv_camera"
      type="cv_camera_node"
      name="right" 
      args="$(arg cam2)" >
  </node>

and replace the values of the following arguments

.. code-block:: xml

  <arg name="cam1" value="0" />
  <arg name="cam2" value="1" />

with the real device ids of the camera (they can be found from the linux filesystem)


Parameters tuning
-----------------

To live-edit the parameters of the stereo image processing run:

.. code-block:: bash

  $ rosrun rqt_reconfigure rqt_reconfigure
