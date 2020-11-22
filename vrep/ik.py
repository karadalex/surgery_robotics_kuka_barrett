import sim
import sys
import ctypes as ct
from math import pi, cos, sin
from time import sleep


def main():
  samplesNum = 20
  step = 1.0 / samplesNum
  t = 0
  for i in range(samplesNum):
    x = 0.1 * cos(2*pi*t) + 0.75
    y = 0.1 * sin(2*pi*t) - 0.125
    z = 1.2
    yaw = 0
    pitch = pi/2
    roll = 0
    pose = [x, y, z] + eulerToQuaternion(yaw, pitch, roll)
    moveTargetFrame(pose)
    t = step*i
    sleep(0.5)


def moveTargetFrame(targetPosition):
  """[summary]

  Args:
      targetPosition ([type]): [description]
  """
  emptyBuff = bytearray()

  res,retInts,target1Pose,retStrings,retBuffer = sim.simxCallScriptFunction(
    clientID,
    'target',
    sim.sim_scripttype_childscript,
    'commandTarget',
    [],targetPosition,[],emptyBuff,
    sim.simx_opmode_oneshot_wait
  )
  print(res,retInts,target1Pose,retStrings,retBuffer)


def eulerToQuaternion(yaw, pitch, roll):
  """[summary]

  Args:
      yaw ([type]): [description]
      pitch ([type]): [description]
      roll ([type]): [description]

  Returns:
      [type]: [description]
  """
  cy = cos(yaw * 0.5)
  sy = sin(yaw * 0.5)
  cp = cos(pitch * 0.5)
  sp = sin(pitch * 0.5)
  cr = cos(roll * 0.5)
  sr = sin(roll * 0.5)

  qw = cr * cp * cy + sr * sp * sy
  qx = sr * cp * cy - cr * sp * sy
  qy = cr * sp * cy + sr * cp * sy
  qz = cr * cp * sy - sr * sp * cy

  return [qx, qy, qz, qw]


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
