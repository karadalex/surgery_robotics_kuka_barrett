#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from control_msgs.msg import JointTrajectoryControllerState


t = []
q1 = []
q2 = []
q3 = []
q4 = []
q5 = []
q6 = []
q7 = []

fig, axs = plt.subplots(4, 2)

def callback(data):
  # time = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/float((10**9))
  time = rospy.get_time()
  th1 = data.actual.positions[0]
  th2 = data.actual.positions[1]
  th3 = data.actual.positions[2]
  th4 = data.actual.positions[3]
  th5 = data.actual.positions[4]
  th6 = data.actual.positions[5]
  th7 = data.actual.positions[6]

  # Some example data to display
  t.append(time)
  q1.append(th1)
  q2.append(th2)
  q3.append(th3)
  q4.append(th4)
  q5.append(th5)
  q6.append(th6)
  q7.append(th7)

  if len(t) > 200:
    t.pop(0)
    q1.pop(0)
    q2.pop(0)
    q3.pop(0)
    q4.pop(0)
    q5.pop(0)
    q6.pop(0)
    q7.pop(0)

  axs[0, 0].clear()
  axs[0, 1].clear()
  axs[1, 0].clear()
  axs[1, 1].clear()
  axs[2, 0].clear()
  axs[2, 1].clear()
  axs[3, 0].clear()

  axs[0, 0].set_title('Joint 1')
  axs[0, 1].set_title('Joint 2')
  axs[1, 0].set_title('Joint 3')
  axs[1, 1].set_title('Joint 4')
  axs[2, 0].set_title('Joint 5')
  axs[2, 1].set_title('Joint 6')
  axs[3, 0].set_title('Joint 7')

  # 30 seconds window

  axs[0, 0].set_xlim(time-30,time)
  axs[0, 0].set_ylim(-3.14, 3.14)
  axs[0, 0].plot(t, q1)
  
  axs[0, 1].set_xlim(time-30,time)
  axs[0, 1].set_ylim(-3.14, 3.14)
  axs[0, 1].plot(t, q2)
  
  axs[1, 0].set_xlim(time-30,time)
  axs[1, 0].set_ylim(-3.14, 3.14)
  axs[1, 0].plot(t, q3)
  
  axs[1, 1].set_xlim(time-30,time)
  axs[1, 1].set_ylim(-3.14, 3.14)
  axs[1, 1].plot(t, q4)

  axs[2, 0].set_xlim(time-30,time)
  axs[2, 0].set_ylim(-3.14, 3.14)
  axs[2, 0].plot(t, q5)

  axs[2, 1].set_xlim(time-30,time)
  axs[2, 1].set_ylim(-3.14, 3.14)
  axs[2, 1].plot(t, q6)

  axs[3, 0].set_xlim(time-30,time)
  axs[3, 0].set_ylim(-3.14, 3.14)
  axs[3, 0].plot(t, q7)

  for ax in axs.flat:
    ax.set(xlabel='seconds', ylabel='angle')

  # Hide x labels and tick labels for top plots and y ticks for right plots.
  for ax in axs.flat:
    ax.label_outer()

  fig.canvas.draw()


def main(args):
  rospy.init_node('plot_trajectories', anonymous=True)
  subscriber = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, callback, queue_size=1)
  plt.show()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)