import sim
import sys
import ctypes as ct
from math import pi, cos, sin
from time import sleep
from ik import *


def main():
  # Execute circular trajectory
  # samplesNum = 20
  # step = 1.0 / samplesNum
  # t = 0
  # for i in range(samplesNum+1):
  #   x = 0.1 * cos(2*pi*t) + 0.75
  #   y = 0.1 * sin(2*pi*t) - 0.125
  #   z = 1.2
  #   yaw = 0
  #   pitch = pi/2
  #   roll = 0
  #   pose = [x, y, z] + eulerToQuaternion(yaw, pitch, roll)
  #   moveTargetFrame(pose, clientID)
  #   t = step*i
  #   sleep(0.5)

  # Execute linear trajectories
  waypoints = [
    Pose(0.85, -0.125, 1.2, 0, pi/2, 0),
    Pose(-0.15, 0.65, 1.2, 0, pi/2, 0),
    Pose(-0.0178, 0.7, 1.6048, 0, pi/2, 0),
  ]
  for i in range(1, len(waypoints)):
    start = waypoints[i-1]
    end = waypoints[i]
    executeLinearTrajectory(start, end, clientID)
    


if __name__ == "__main__":
  # Establish Connection
  sim.simxFinish(-1)
  clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)

  if clientID != -1:
    print("Connected to remote API Server")      
  else:
    print("Connection not succesfull")
    sys.exit("Could not connect")

  main()
