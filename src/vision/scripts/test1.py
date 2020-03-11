#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('vision')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/opencv/test_topic_1",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/barrett/camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    blueColor = (255,0,0)

    # Define and draw detection area
    detectionAreaXRange = (int(rows/3), int(2*rows/3))
    detectionAreaYRange = (int(cols/3), int(2*cols/3))

    # Count blue pixels within detection area
    numBluePixels = 0
    for i in range(detectionAreaXRange[0], detectionAreaXRange[1]):
      for j in range(detectionAreaYRange[0], detectionAreaYRange[1]):
        if cv_image[i,j,0] >= 200:
          numBluePixels += 1

    # Decide if there are enough blue pixels to consider them a blue tool
    if numBluePixels >= 20:
      detectionMsg = "Blue tool detected"
    else:
      detectionMsg = "No tool detected"

    # Draw detection message
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    topLeftCorner = (50,50)
    fontScale              = 1
    lineType               = 2
    cv2.putText(
      cv_image,
      detectionMsg, 
      topLeftCorner, 
      font, 
      fontScale,
      blueColor,
      lineType
    )

    # Draw detection area
    cv2.rectangle(
      cv_image,
      (detectionAreaYRange[0], detectionAreaXRange[0]),  # top left corner
      (detectionAreaYRange[1], detectionAreaXRange[1]),  # bottom right corner
      blueColor,
      2 # 2px thickness
    )

    cv2.imshow("OpenCV Image", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  



def main(args):
  ic = image_converter()
  rospy.init_node('opencv_image_view', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)