#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import cm
from matplotlib.ticker import LinearLocator
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray
import math
import random as rand # TO BE REMOVED later


qi_min = [-3.14 for i in range(7)]
qi_max = [3.14 for i in range(7)]
x = []
y = []
z = []

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


def callback(data):
  manipulability = manipulability_measure(data.data)
  Lq = joint_limit_measure()
  manipulability_plot()


def manipulability_measure(array):
  jacobian_array = np.array(array)
  jac = np.resize(jacobian_array, (6,7))
  manipulability = math.sqrt(np.linalg.det(np.matmul(jac, np.transpose(jac))))
  return manipulability


def joint_limit_measure():
  return 1


def manipulability_plot():
  # Make data.
  x.append(rand.gauss(0, 1))
  y.append(rand.gauss(0, 1))
  z.append(rand.gauss(0, 1))

  # Plot the points
  ax.clear()
  ax.scatter(x, y, z)
  fig.canvas.draw()


def main(args):
  rospy.init_node('manipulability_plot', anonymous=True)

  subscriber = rospy.Subscriber("/jacobian_state", Float64MultiArray, callback, queue_size=1)

  manipulability_plot()
  plt.show()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)