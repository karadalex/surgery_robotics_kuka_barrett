#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from helpers import *
import geometry


roslib.load_manifest('vision')


class Detection:

  def __init__(self):
    self.image_pub = rospy.Publisher("/opencv/test_topic_1",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/stereo/left/image_raw",Image,self.callback)

    # Data to be available across every subscriber callback
    self.toolCenterOfMass = [0, 0]
    self.prevFrameTime = time.time()
    self.toolOrientX = []
    self.toolOrientY = []

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape

    blueColor = (255,0,0)
    greenColor = (0,255,0)
    redColor = (0,0,255)

    ###########################################################################################
    #    COLOR DETECTION (B,G,R)
    ###########################################################################################
    # Define color detection area
    detectionAreaYRange = (int(rows/3), int(2*rows/3))
    detectionAreaXRange = (int(cols/3), int(2*cols/3))
    colorROI = np.array([
      [detectionAreaXRange[0], 0],
      [detectionAreaXRange[1], 0],
      [detectionAreaXRange[1], rows],
      [detectionAreaXRange[0], rows]
    ])

    # Count blue and/or green pixels within detection area
    numBluePixels = 0
    numGreenPixels = 0
    toolPixels = []
    # for i in range(detectionAreaYRange[0], detectionAreaYRange[1], 3):
    for i in range(0, rows, 10):
      # for j in range(0, cols, 10):
      for j in range(detectionAreaXRange[0], detectionAreaXRange[1], 10):
        if cv_image[i,j,0] >= 200:
          numBluePixels += 1
          cv_image[i,j] = [0,255,0]
          toolPixels.append(np.array([[i,j]]))
        elif cv_image[i,j,1] >= 180:
          numGreenPixels += 1
    toolPixels = np.array(toolPixels)

    # Decide if there are enough blue pixels to consider them a blue tool
    if numBluePixels >= 20:
      toolDetectionMsg = "Blue tool detected"
      blueToolDetected = True
    else:
      toolDetectionMsg = "No tool detected"
      blueToolDetected = False
    # Decide if there are enough blue pixels to consider them a mounting dock containing a Trocar
    if numGreenPixels >= 20:
      trocarDetectionMsg = "Trocar detected"
      trocarDetected = True
    else:
      trocarDetectionMsg = "No Trocar detected"
      trocarDetected = False


    ###########################################################################################
    #    SHAPE DETECTION
    ###########################################################################################
    # Restrict detection in the center columns
    # img_detection_region = cv_image[0:rows, int(cols/3):int(2*cols/3)]
    img_detection_region = cv_image

    # convert to grayscale
    gray = cv2.cvtColor(img_detection_region, cv2.COLOR_BGR2GRAY)

    # make boundary of image white so that contours will be always closed loops within the region
    gray[0:img_detection_region.shape[0], 0].fill(255)
    gray[0:img_detection_region.shape[0], img_detection_region.shape[1]-1].fill(255)
    gray[0, 0:img_detection_region.shape[1]].fill(255)
    gray[img_detection_region.shape[0]-1, 0:img_detection_region.shape[1]].fill(255)

    # blur the image
    blur = cv2.blur(gray, (3, 3))
    # thresholded image for blue color
    ret, thresh = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY)
    # thresholded image for green color
    ret, threshGreen = cv2.threshold(blur, 110, 255, cv2.THRESH_BINARY)

    # Finding contours for the thresholded image
    im2, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    im2, contoursGreen, hierarchy = cv2.findContours(threshGreen, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # create hull array for convex hull points
    hull = []
    # calculate points for each contour
    for i in range(len(contours)):
      # creating convex hull object for each contour
      hull.append(cv2.convexHull(contours[i], False))

    # Calculate and store the center of mass of each contour
    contourCenterOfMass = []
    for i in range(len(contours)):
      contourCenterOfMass.append(geometry.centerOfMass(contours[i]))

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
        if hasPolygonCenterOfColor(cv_image, contours[i], blueColor):
          if isPolygonInROI(hull[i], colorROI):
            maxHullIndex = i


    ###########################################################################################
    #    POSE DETECTION
    ###########################################################################################
    # Center of mass of tool (weighted average of cm computed from contour and cm computed from toolPixels)
    toolPixelsCM = geometry.centerOfMass(toolPixels) # toolPixelsCM coordinates are (Y,X) (??)
    maxHullCmX = (5*contourCenterOfMass[maxHullIndex][0] + 1*toolPixelsCM[1]) / 6
    maxHullCmY = (5*contourCenterOfMass[maxHullIndex][1] + 1*toolPixelsCM[0]) / 6

    # Find orientation vectors of tool
    a,b = geometry.orientationVectors(toolPixels)
    # attach vectors to center of mass point
    a = a + [maxHullCmX, maxHullCmY]
    b = b + [maxHullCmX, maxHullCmY]

    # Update toolCenterOfMass point variable (and orientation) only if there is a significant change in (x,y) values,
    # in order to make the point and orientation stable at all times
    cmChangeThreshold = 2
    if (abs(self.toolCenterOfMass[0] - maxHullCmX) > cmChangeThreshold) and (abs(self.toolCenterOfMass[1] - maxHullCmY) > cmChangeThreshold):
      self.toolCenterOfMass[0] = maxHullCmX
      self.toolCenterOfMass[1] = maxHullCmY
      self.toolOrientX = a
      self.toolOrientY = b


    ###########################################################################################
    #    FIND GRASP POINTS - FORCE CLOSURE
    ###########################################################################################
    # TODO

    ###########################################################################################
    #    DRAW OPENCV IMAGE
    ###########################################################################################
    # draw contour and hull points of the biggest convex hull, 
    # Draw if there was a blue tool
    contour_color = (0, 0, 255)
    convex_hull_color = (0, 255, 0)
    if len(hull) > 1 and blueToolDetected: # if there is only one hull, it means this is the while picture (default convex hull)
      # draw max convex hull object
      cv2.drawContours(img_detection_region, hull, maxHullIndex, convex_hull_color, 2, 8)
      # draw center of mass of convex hull
      cmX = self.toolCenterOfMass[0]
      cmY = self.toolCenterOfMass[1]
      cv2.circle(img_detection_region,(cmX,cmY),5,(0,0,255),-1)
      # draw orientation vectors
      a = self.toolOrientX
      b = self.toolOrientY
      cv2.arrowedLine(img_detection_region, (cmX,cmY), (a[0], a[1]), redColor, 2, 8, 0, 0.1)
      cv2.arrowedLine(img_detection_region, (cmX,cmY), (b[0], b[1]), greenColor, 2, 8, 0, 0.1)
    if trocarDetected:
      for i in range(1, len(contoursGreen)):
      # draw ith contour
        cv2.drawContours(img_detection_region, contoursGreen, i, contour_color, 2, 8, hierarchy)
    cmb = geometry.centerOfMass(toolPixels)

    # Draw Color Region Of Interest
    # cv2.polylines(cv_image, [colorROI], True, (0,255,0), thickness=3)

    # Draw tool detection message
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    topLeftCorner = (50,50)
    fontScale              = 0.5
    lineType               = 1
    cv2.putText(
      cv_image,
      toolDetectionMsg, 
      topLeftCorner, 
      font, 
      fontScale,
      blueColor,
      lineType
    )

    # Draw trocar detection message
    position = (50,70)
    cv2.putText(
      cv_image,
      trocarDetectionMsg, 
      position, 
      font, 
      fontScale,
      greenColor,
      lineType
    )

    # Calculate and display FPS
    endOfFrameTime = time.time()
    seconds = endOfFrameTime - self.prevFrameTime
    self.prevFrameTime = endOfFrameTime
    fps = round(1/seconds, 2)
    fpsInfo = "FPS = {}".format(fps)
    position = (cols-200,50)
    cv2.putText(
      cv_image,
      fpsInfo, 
      position, 
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
  ic = Detection()
  rospy.init_node('opencv_image_view', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)