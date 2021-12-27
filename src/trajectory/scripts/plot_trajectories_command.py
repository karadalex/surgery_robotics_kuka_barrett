#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from trajectory_msgs.msg import JointTrajectory


# fig, axs = plt.subplots(4, 2)
fig = plt.figure()

def callback(data):
  waypoints_num = len(data.points)

  t = np.zeros((1, waypoints_num))
  q = np.zeros((7, waypoints_num))
  qd = np.zeros((7, waypoints_num))
  qdd = np.zeros((7, waypoints_num))

  for i in range(waypoints_num):
    point = data.points[i]
    t[0, i] = point.time_from_start.secs
    for j in range(7):
      q[j, i] = point.positions[j]
      qd[j, i] = point.velocities[j]
      qdd[j, i] = point.accelerations[j]

  for j in range(7):
    ax = plt.subplot(4, 2, j + 1)

    ax.clear()
    ax.set_title("Joint "+str(j+1))
    ax.set(xlabel='seconds', ylabel='angle')
    ax.label_outer()

    ax.plot(t[0, :], q[j, :], label="q"+str(j+1), marker="o", ms=3, color="tab:blue")
    ax.legend()

    qdax = ax.twinx()
    qdax.set(xlabel='seconds', ylabel='velocity/acceleration')
    qdax.plot(t[0, :], qd[j, :], label="qd"+str(j+1), marker="o", ms=3, color="tab:orange", alpha=0.2)
    qdax.legend()

    qdax.plot(t[0, :], qdd[j, :], label="qdd"+str(j+1), marker="o", ms=3, color="tab:red", alpha=0.2)
    qdax.legend()

  fig.canvas.draw()


def main(args):
  rospy.init_node('plot_trajectories_command', anonymous=True)
  subscriber = rospy.Subscriber("/arm_controller/command", JointTrajectory, callback, queue_size=1)
  plt.show()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)