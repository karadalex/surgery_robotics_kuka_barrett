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
  targetPosition = {0,0,0,0,0,0,0}

  commandIIWA = function(inInts,inFloats,inStrings,inBuffer)
    robotFlag = true
    if #inFloats == 7 then
      targetPosition = inFloats
    end
    return {},{},{'Robot received command'},'' -- return a string
  end

  -- Put your main loop here, e.g.:
  --
  while sim.getSimulationState() ~= sim.simulation_advancing_abouttostop do
    if robotFlag then
      moveIIWA(targetPosition)
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

function moveIIWA(targetPosition)
  jointHandles={-1,-1,-1,-1,-1,-1,-1}
  for i=1,7,1 do
    jointHandles[i]=sim.getObjectHandle('LBR_iiwa_14_R820_joint'..i)
  end

  -- Set-up some of the RML vectors:
  vel=110
  accel=40
  jerk=80
  currentVel={0,0,0,0,0,0,0}
  currentAccel={0,0,0,0,0,0,0}
  maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
  maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
  maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
  targetVel={0,0,0,0,0,0,0}

  sim.rmlMoveToJointPositions(
    jointHandles,-1,
    currentVel,currentAccel,
    maxVel,maxAccel,maxJerk,
    targetPosition,targetVel
  )
end