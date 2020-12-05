import sim
import sys
import ctypes as ct
from math import pi, cos, sin
from time import sleep


class Pose:
  def __init__(self, x, y, z, roll, pitch, yaw):
    """[summary]

    Args:
        x (float): [description]
        y (float): [description]
        z (float): [description]
        roll (float): [description]
        pitch (float): [description]
        yaw (float): [description]
    """
    self.x = x
    self.y = y
    self.z = z
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw


def executeLinearTrajectory(start, end, clientID, samplesNum=20):
  """[summary]

  Args:
      start ([type]): [description]
      end ([type]): [description]
      clientID ([type]): [description]
      samplesNum (int, optional): [description]. Defaults to 20.
  """
  samplesNum = 20
  step = 1.0 / samplesNum
  t = 0
  for i in range(samplesNum+2):
    x = t*(end.x - start.x) + start.x
    y = t*(end.y - start.y) + start.y
    z = t*(end.z - start.z) + start.z
    yaw = t*(end.yaw - start.yaw) + start.yaw
    pitch = t*(end.pitch - start.pitch) + start.pitch
    roll = t*(end.roll - start.roll) + start.roll
    pose = [x, y, z] + eulerToQuaternion(yaw, pitch, roll)
    moveTargetFrame(pose, clientID)
    t = step*i
    sleep(0.5)


def moveTargetFrame(targetPosition, clientID):
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
