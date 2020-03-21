#!/bin/bash


rostopic pub /gripper_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['bh_j11_joint', 'bh_j12_joint', 'bh_j13_joint', 'bh_j21_joint', 'bh_j22_joint', 'bh_j23_joint', 'bh_j32_joint', 'bh_j33_joint']
points:
- positions: [0,0,0,0,0,0,0,0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 2.0, nsecs: 990099009}"
