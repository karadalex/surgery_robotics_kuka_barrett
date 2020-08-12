Troubleshooting
===============

.. error:: UnicodeDecodeError: 'ascii' codec can't decode byte 0xce in position 33: ordinal not in range(128)

Make sure the project path doesn't contain any greek characters

.. error:: Eigen/Geometry: No such file or directory

Try one of the following commands

.. code-block:: bash

  $ sudo apt-get install libeigen3-dev 
  $ sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen # if the above does not work either

.. error:: gzclient: /build/ogre-1.9-B6QkmW/ogre-1.9-1.9.0+dfsg1/OgreMain/include/OgreAxisAlignedBox.h:252: void Ogre::AxisAlignedBox::setExtents(const Ogre::Vector3&, const Ogre::Vector3&): Assertion `(min.x <= max.x && min.y <= max.y && min.z <= max.z) && "The minimum corner of the box must be less than or equal to maximum corner"' failed.

Make sure to upgrade gazebo (ros-melodic-gazebo-ros-pkgs) to the latest version
