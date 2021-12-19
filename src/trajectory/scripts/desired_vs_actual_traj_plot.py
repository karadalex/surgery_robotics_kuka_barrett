#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import matplotlib.pyplot as plt
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import Axes3D
from kinematics.msg import DesiredForwardKinematicsTrajectory, KinematicState


# Desired trajectory
xd = []
yd = []
zd = []

# Actual trajectory
x = []
y = []
z = []

def desired_traj_callback(data):
  for point in data.desired_fwd_traj:
    position = point.transform.translation
    xd.append(position.x)
    yd.append(position.y)
    zd.append(position.z)
  
  plot_all()

def actual_state_callback(data):
  position = data.transform.transform.translation
  x.append(position.x)
  y.append(position.y)
  z.append(position.z)

  plot_all()
  

def plot_all():
  ax.clear()
  ax.plot3D(xd, yd, zd)
  ax.scatter(x, y, z, c="r")

  try:
    fig.canvas.draw()
  except:
    pass


def main(args):
  global fig
  global ax
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  rospy.init_node('desired_vs_actual_traj_plot', anonymous=True)

  subscriber1 = rospy.Subscriber("/desired_robot_trajectory_fwd", DesiredForwardKinematicsTrajectory, desired_traj_callback, queue_size=1)
  subscriber2 = rospy.Subscriber("/kinematic_state", KinematicState, actual_state_callback, queue_size=1)
  
  # rospy.spin()
  plt.show(block=True)

  # try:
  #   rospy.spin()
  # except KeyboardInterrupt:
  #   print("Shutting down")


if __name__ == '__main__':
  main(sys.argv)