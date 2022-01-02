#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import tf


frame_pubs = [None for i in range(4)]
for i in range(4):
  topic = "fulcrum/estimated/frame"+str(i+1)
  frame_pubs[i] = rospy.Publisher(topic, PoseWithCovarianceStamped, queue_size=10)

# Fulcrum reference frames poses in x, y, z, roll, pitch, yaw
poses = [
  [0.526212, 0.213600, 1.341008, -0.477707, 0, 0],
  [0.529996, 0.059271, 1.398114, -0.271542, 0, 0],
  [0.523540, -0.108797, 1.398272, 0.305399, 0, 0],
  [0.523540, -0.259344, 1.335326, 0.557652, 0, 0]
]

rospy.init_node('fulcrum_frames_estimator')
frame_count = 0

r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():

  for (index, pose_array) in enumerate(poses):
    frame = PoseWithCovarianceStamped()
    frame_pose = Pose()

    frame_pose.position.x = pose_array[0]
    frame_pose.position.y = pose_array[1]
    frame_pose.position.z = pose_array[2]

    frame_quaternion = tf.transformations.quaternion_from_euler(pose_array[3], pose_array[4], pose_array[5])
    frame_pose.orientation.x = frame_quaternion[0]
    frame_pose.orientation.y = frame_quaternion[1]
    frame_pose.orientation.z = frame_quaternion[2]
    frame_pose.orientation.w = frame_quaternion[3]
    frame.pose.pose = frame_pose

    cov = np.diag(np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001]))
    frame.pose.covariance = cov.flatten()

    frame.header.seq = frame_count
    frame.header.stamp = rospy.get_rostime()
    frame.header.frame_id = "world"

    frame_pubs[index].publish(frame)
  
  frame_count += 1
  
  r.sleep()
