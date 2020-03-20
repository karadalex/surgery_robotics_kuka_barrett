#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from helpers import getRectangularNeighborhood


roslib.load_manifest('vision')


class ToolDetection:

  def __init__(self):
    self.image_pub = rospy.Publisher("/opencv/test_topic_1",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/barrett/camera1/image_raw",Image,self.callback)

    # Data to be available across every subscriber callback
    self.toolCenterOfMass = [0, 0]

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    blueColor = (255,0,0)

    ###########################################################################################
    #    COLOR DETECTION
    ###########################################################################################
    blueColor = (255,0,0)
    # Define color detection area
    detectionAreaXRange = (int(rows/3), int(2*rows/3))
    detectionAreaYRange = (int(cols/3), int(2*cols/3))
    # Count blue pixels within detection area
    numBluePixels = 0
    for i in range(detectionAreaXRange[0], detectionAreaXRange[1], 3):
      for j in range(detectionAreaYRange[0], detectionAreaYRange[1], 3):
        if cv_image[i,j,0] >= 200:
          numBluePixels += 1
    # Decide if there are enough blue pixels to consider them a blue tool
    if numBluePixels >= 20:
      detectionMsg = "Blue tool detected"
      blueToolDetected = True
    else:
      detectionMsg = "No tool detected"
      blueToolDetected = False


    ###########################################################################################
    #    SHAPE DETECTION
    ###########################################################################################
    # Restrict detection in the center columns
    img_detection_region = cv_image[0:rows, int(cols/3):int(2*cols/3)]
    # img_detection_region = cv_image

    # convert to grayscale
    gray = cv2.cvtColor(img_detection_region, cv2.COLOR_BGR2GRAY)

    # make boundary of image white so that contours will be always closed loops within the region
    gray[0:img_detection_region.shape[0], 0].fill(255)
    gray[0:img_detection_region.shape[0], img_detection_region.shape[1]-1].fill(255)
    gray[0, 0:img_detection_region.shape[1]].fill(255)
    gray[img_detection_region.shape[0]-1, 0:img_detection_region.shape[1]].fill(255)

    # blur the image
    blur = cv2.blur(gray, (3, 3))
    ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)

    # Finding contours for the thresholded image
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # create hull array for convex hull points
    hull = []
    # calculate points for each contour
    for i in range(len(contours)):
      # creating convex hull object for each contour
      hull.append(cv2.convexHull(contours[i], False))

    # Calculate and store the center of mass of each contour
    contourCenterOfMass = []
    for i in range(len(contours)):
      sumX = 0
      sumY = 0
      for point in contours[i]:
        sumX += point[0][0]
        sumY += point[0][1]
      avgX = int(sumX/len(contours[i]))
      avgY = int(sumY/len(contours[i]))
      contourCenterOfMass.append([avgX, avgY])

    # For each convex hull, calculate mean distance of hull points from center of mass
    hullsMeanDistanceFromCenter = []
    for i in range(len(hull)):
      sumD = 0
      for point in hull[i]:
        cm = contourCenterOfMass[i]
        distance = math.sqrt((point[0][0] - cm[0])**2 + (point[0][1] - cm[1])**2)
        sumD += distance
      avgD = sumD/len(hull[i])
      hullsMeanDistanceFromCenter.append(avgD)
    # Calculate max convex hull by finding the maximum mean distance
    if len(hull) > 1:
      startFrom = 1 # Ignore first convex hull (if there are many), because it is the whole picture
    else:
      startFrom = 0
    maxHullIndex = startFrom
    for i in range(startFrom, len(hull)):
      if hullsMeanDistanceFromCenter[i] > hullsMeanDistanceFromCenter[maxHullIndex]:
        maxHullIndex = i

    # Update toolCenterOfMass point variable only if there is a significant change in (x,y) values,
    # in order to make the point stable at all times
    maxHullCmX = contourCenterOfMass[maxHullIndex][0]
    maxHullCmY = contourCenterOfMass[maxHullIndex][1]
    cmChangeThreshold = 2
    if (abs(self.toolCenterOfMass[0] - maxHullCmX) > cmChangeThreshold) and (abs(self.toolCenterOfMass[1] - maxHullCmY) > cmChangeThreshold):
      self.toolCenterOfMass[0] = maxHullCmX
      self.toolCenterOfMass[1] = maxHullCmY


    ###########################################################################################
    #    FIND GRASP POINTS - FORCE CLOSURE
    ###########################################################################################
    # TODO

    ###########################################################################################
    #    DRAW OPENCV IMAGE
    ###########################################################################################
    # draw contour and hull points of the biggest convex hull, 
    # Draw if there was a blue tool detected
    if len(hull) > 1 and blueToolDetected : # if there is only one hull, it means this is the while picture (default convex hull)
      contour_color = (0, 0, 255)
      convex_hull_color = (0, 255, 0)
      # draw ith contour
      # cv2.drawContours(img_detection_region, contours, maxHullIndex, contour_color, 2, 8, hierarchy)
      # draw ith convex hull object
      cv2.drawContours(img_detection_region, hull, maxHullIndex, convex_hull_color, 2, 8)
      # draw center of mass of convex hull
      cmX = self.toolCenterOfMass[0]
      cmY = self.toolCenterOfMass[1]
      cv2.circle(img_detection_region,(cmX,cmY),5,(0,0,255),-1)

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

    cv2.imshow("OpenCV Image", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)
  



def main(args):
  ic = ToolDetection()
  rospy.init_node('opencv_image_view', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)