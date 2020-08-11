#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import Axes3D
import math
from kinematics.msg import KinematicState


qi_min = [-3.14 for i in range(7)]
qi_max = [3.14 for i in range(7)]
x = []
y = []
z = []
cs = []

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
isColorbarShown = False
cmap = mpl.cm.coolwarm_r
norm = mpl.colors.Normalize(vmin=0, vmax=1)
points = ax.scatter(x, y, z, c=cs, cmap=cmap)


def callback(data):
  manipulability = manipulability_measure(data.jacobian.data)
  Lq = joint_limit_measure()
  manipulability_plot(data.transform.transform.translation, manipulability)
  print(manipulability)


def manipulability_measure(array):
  jacobian_array = np.array(array)
  jac = np.resize(jacobian_array, (6,7))
  manipulability = math.sqrt(np.linalg.det(np.matmul(jac, np.transpose(jac))))
  return manipulability


def joint_limit_measure():
  return 1


def manipulability_plot(position, manipulability):
  # Make data.
  x.append(position.x)
  y.append(position.y)
  z.append(position.z)
  cs.append(manipulability)

  # Plot the points
  ax.clear()
  points = ax.scatter(x, y, z, c=cs, cmap=cmap, norm=norm)

  global isColorbarShown
  if not isColorbarShown:
    fig.colorbar(points)
    isColorbarShown = True
  fig.canvas.draw()

def main(args):
  rospy.init_node('manipulability_plot', anonymous=True)

  subscriber = rospy.Subscriber("/kinematic_state", KinematicState, callback, queue_size=1)

  # fig.colorbar(points)
  plt.show()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)