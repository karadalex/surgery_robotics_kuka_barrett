import sim
import sys
import ctypes as ct
from math import pi


def main():  
  position1 = [pi/2,0,0,0,0,0,0]
  moveArmToJointsPosition(position1)


def moveArmToJointsPosition(targetPosition):
  emptyBuff = bytearray()

  res,retInts,target1Pose,retStrings,retBuffer = sim.simxCallScriptFunction(
    clientID,
    'LBR_iiwa_14_R820',
    sim.sim_scripttype_childscript,
    'commandIIWA',
    [],targetPosition,[],emptyBuff,
    sim.simx_opmode_oneshot_wait
  )


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
