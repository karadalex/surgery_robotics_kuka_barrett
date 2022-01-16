#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from std_msgs.msg import Float64
from csv import writer


errors = []

def callback(data):
  global errors
  now = rospy.get_rostime()
  # push value in array and in every 100 values
  rospy.loginfo("RCM error: "+str(now.nsecs)+" "+str(data.data))
  errors.append([now.nsecs, data.data])

  if len(errors) == 10:
    with open('rcm_error_data.csv', 'a') as csv_file:
      writer_object = writer(csv_file)
      writer_object.writerows(errors)
      csv_file.close()
    errors = []


def main(args):
  rospy.init_node('rcm_error_save_csv', anonymous=True)

  subscriber = rospy.Subscriber("/fulcrum/error", Float64, callback, queue_size=1)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)