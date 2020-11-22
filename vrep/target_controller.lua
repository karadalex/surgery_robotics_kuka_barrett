-- if you wish to execute code contained in an external file instead,
-- use the require-directive, e.g.:
--
-- require 'myExternalFile'
--
-- Above will look for <V-REP executable path>/myExternalFile.lua or
-- <V-REP executable path>/lua/myExternalFile.lua
-- (the file can be opened in this editor with the popup menu over
-- the file name)

function sysCall_threadmain()
  -- Put some initialization code here
  simRemoteApi.start(19999)
  robotFlag = false
  targetPose = {0.5, -0.125, 2.2, 0, 0, 0, 1}

  commandTarget = function(inInts,inFloats,inStrings,inBuffer)
    robotFlag = true
    if #inFloats == 7 then
      targetPose = inFloats
    end
    return {},{},{'Robot received command'},'' -- return a string
  end

  -- Put your main loop here, e.g.:
  --
  while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
    if robotFlag then
      moveTarget(targetPose)
    end
    sim.switchThread() -- resume in next simulation step
  end
end

function sysCall_cleanup()
  -- Put some clean-up code here
end


-- ADDITIONAL DETAILS:
-- -------------------------------------------------------------------------
-- If you wish to synchronize a threaded loop with each simulation pass,
-- enable the explicit thread switching with 
--
-- sim.setThreadAutomaticSwitch(false)
--
-- then use
--
-- sim.switchThread()
--
-- When you want to resume execution in next simulation step (i.e. at t=t+dt)
--
-- sim.switchThread() can also be used normally, in order to not waste too much
-- computation time in a given simulation step
-- -------------------------------------------------------------------------

function moveTarget(targetPose)
  targetHandle = sim.getObjectHandle('target')
  relativeToObjectHandle = -1
  flags = -1

  targetPosition = {targetPose[1], targetPose[2], targetPose[3]}
  targetQuaternion = {targetPose[4], targetPose[5], targetPose[6], targetPose[7]}
  -- targetQuaternion = nil
  -- print("targetPosition: ", targetPosition, "targetQuaternion: ", targetQuaternion)

  vel = 0.2
  accel = 2
  jerk = 10
  currentVel = {0,0,0,0}
  currentAccel = {0,0,0,0}
  maxVel = {vel, vel, vel, 1}
  maxAccel = {accel, accel, accel, 1}
  maxJerk = {jerk, jerk, jerk, 1}
  targetVel = nil

  -- sim.setObjectPosition(targetHandle, -1, targetPosition)
  -- sim.setObjectOrientation(targetHandle, -1, targetQuaternion)
  
  result, newPos, newQuaternion, newVel, newAccel, timeLeft = sim.rmlMoveToPosition(
    targetHandle, relativeToObjectHandle, 
    flags, 
    currentVel, currentAccel, 
    maxVel, maxAccel, maxJerk,
    targetPosition, targetQuaternion, targetVel
  )
  print(timeLeft)
end