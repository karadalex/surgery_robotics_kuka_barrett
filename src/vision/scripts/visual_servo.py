#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
import math
import time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from helpers import *
import geometry


roslib.load_manifest('vision')

disable_servo = False
show_error_graph = False


class Detection:

  def __init__(self):
    self.image_pub = rospy.Publisher("/opencv/test_topic_1", Image)
    self.servo_cmd = rospy.Publisher("kuka_barrett/cmd_vel", Twist)
    self.error_pub = rospy.Publisher("kuka_barrett/visual_servo/error", Float32)
    self.error_theta_pub = rospy.Publisher("kuka_barrett/visual_servo/error_theta", Float32)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/stereo/left/image_raw",Image,self.callback)

    # Data to be available across every subscriber callback
    self.toolCenterOfMass = [0, 0]
    self.prevFrameTime = time.time()
    self.toolOrientX = []
    self.toolOrientY = []

    # Setup visual servoing controller
    self.Kp = rospy.get_param("/visual_servo/controller/p", 0.9)
    self.Ki = rospy.get_param("/visual_servo/controller/i", 0.01)
    self.Kd = rospy.get_param("/visual_servo/controller/d", 0.2)
    self.prev_ex = 0
    self.ex_sum = 0
    self.prev_ey = 0
    self.ey_sum = 0
    self.prev_eth = 0
    self.eth_sum = 0
    self.error_tolerance = 0.001
    self.prev_error_norm = 0.0

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
    # for i in range(detectionAreaYRange[0], detectionAreaYRange[1], 3):
    for i in range(0, rows, 10):
      # for j in range(0, cols, 10):
      for j in range(detectionAreaXRange[0], detectionAreaXRange[1], 10):
        if cv_image[i,j,0] >= 200:
          numBluePixels += 1
          # cv_image[i,j] = [0,255,0]
        elif cv_image[i,j,1] >= 240:
          numGreenPixels += 1

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
      if hasPolygonCenterOfColor(cv_image, contours[i], blueColor):
        if hullsMeanDistanceFromCenter[i] > hullsMeanDistanceFromCenter[maxHullIndex]:
          maxHullIndex = i


    ###########################################################################################
    #    POSE DETECTION
    ###########################################################################################

    # Calculate ROI of selected tool using it's convex hull
    min_x = cols
    min_y = rows
    max_x = max_y = 0
    for point in hull[maxHullIndex]:
      if point[0][0] < min_x:
        min_x = point[0][0]
      if point[0][1] < min_y:
        min_y = point[0][1]
      if point[0][0] > max_x:
        max_x = point[0][0]
      if point[0][1] > max_y:
        max_y = point[0][1]

    toolROI = np.array([
      [min_x, min_y],
      [max_x, min_y],
      [max_x, max_y],
      [min_x, max_y]
    ])
    
    # Get pixels inside the selected contour
    # CAUTION: This is very heavy computanionally and drops FPS by half or more!
    # Could be avoided with a more clever handling
    toolPixels = []
    # for i in range(detectionAreaYRange[0], detectionAreaYRange[1], 3):
    for i in range(min_y, max_y, 10):
      for j in range(min_x, max_x, 10):
      # for j in range(detectionAreaXRange[0], detectionAreaXRange[1], 10):
        point = (j,i)
        if cv2.pointPolygonTest(contours[maxHullIndex], point, False) != -1:
          numBluePixels += 1
          cv_image[i,j] = [0,255,0]
          toolPixels.append(np.array([[i,j]]))
        elif cv_image[i,j,1] >= 240:
          numGreenPixels += 1
    toolPixels = np.array(toolPixels)

    # Center of mass of tool (weighted average of cm computed from contour and cm computed from toolPixels)
    toolPixelsCM = geometry.centerOfMass(toolPixels) # toolPixelsCM coordinates are (Y,X) (??)
    maxHullCmX = (contourCenterOfMass[maxHullIndex][0] + 5*toolPixelsCM[1]) / 6
    maxHullCmY = (contourCenterOfMass[maxHullIndex][1] + 5*toolPixelsCM[0]) / 6

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
    #    VISUAL SERVOING COMMAND
    ###########################################################################################
    # The origin is the desired position for our visual servo controller
    originY = int(rows/2)
    originX = int(cols/2)

    # Actual position of center of mass
    cmX = self.toolCenterOfMass[0]
    cmY = self.toolCenterOfMass[1]

    if not disable_servo and blueToolDetected:
      ex = round(0.1*(cmX - originX)/float(cols), 4)
      ey = -round(0.1*(cmY - originY)/float(rows), 4)
      eth = round(math.atan(a[0]/a[1]), 4)/math.pi
      
      error_norm = math.sqrt(ex**2 + ey**2)
      # Filter out sudden spikes in error which occur from sudden temporary detection of another tool
      if error_norm > 2*self.prev_error_norm and self.prev_error_norm > 0.0:
        ex = self.prev_ex
        ey = self.prev_ey
        error_norm = self.prev_error_norm
      
      twist = Twist()
      # Only send the command if one of the errors is bigger than the allowed tolerance, so that
      # we avoid small oscillations in the target position
      if error_norm > self.error_tolerance or eth > self.error_tolerance:
        twist.linear.x = self.Kp*ex + self.Kd*(ex - self.prev_ex)
        twist.linear.y = self.Kp*ey + self.Kd*(ey - self.prev_ey)
        # twist.angular.z = eth
        self.servo_cmd.publish(twist)
      
      self.error_pub.publish(error_norm)
      self.error_theta_pub.publish(eth)

      self.prev_ex = ex
      self.ex_sum += ex
      self.prev_ey = ey
      self.ey_sum += ey
      self.prev_error_norm = math.sqrt(self.prev_ex**2 + self.prev_ey**2)

    ###########################################################################################
    #    FIND GRASP POINTS - FORCE CLOSURE
    ###########################################################################################
    
    # create an image filled with zeros, single-channel, same size as img_detection_region.
    blank = np.zeros(img_detection_region.shape[0:2])
    cv2.circle(img_detection_region, (cmX,cmY), 100, 1, 1)
    circle_img = cv2.circle(blank.copy(), (cmX,cmY), 100, 1, 1)
    contour_img = cv2.drawContours(blank.copy(), contours, maxHullIndex, 1, 1, 8)
    intersectedImg = circle_img + contour_img
    intersectionPoints = np.argwhere(intersectedImg == np.max(intersectedImg))
    reducedIntersectionPoints = [intersectionPoints[0]]
    for p in range(1, len(intersectionPoints)):
      pt1 = intersectionPoints[p]
      canAddPoint = True
      for pt2 in reducedIntersectionPoints:
        # Manhattan Distance
        if abs(pt1[0] - pt2[0]) + abs(pt1[1] - pt2[1]) < 10:
          canAddPoint = False
          break
      if canAddPoint:
        reducedIntersectionPoints.append(pt1)
    
    if len(reducedIntersectionPoints) > 3:
      reducedIntersectionPoints = reducedIntersectionPoints[0:3]

    # print("reducedIntersectionPoints", reducedIntersectionPoints)

    ###########################################################################################
    #    DRAW OPENCV IMAGE
    ###########################################################################################

    if not disable_servo:
      # Draw target at the center (origin) of the image
      cv2.circle(img_detection_region,(originX,originY),10,(255,0,255),-1)

    # draw contour and hull points of the biggest convex hull, 
    # Draw if there was a blue tool
    contour_color = (0, 0, 255)
    convex_hull_color = (0, 255, 0)

    # for i in range(len(contours)):
    #   cv2.drawContours(img_detection_region, contours, i, contour_color, 2, 8)

    if len(hull) > 1 and blueToolDetected: # if there is only one hull, it means this is the while picture (default convex hull)
      # draw max convex hull object
      cv2.drawContours(img_detection_region, hull, maxHullIndex, convex_hull_color, 2, 8)

      # draw center of mass of convex hull
      cv2.circle(img_detection_region,(cmX,cmY),5,(0,0,255),-1)

      # draw orientation vectors
      a = self.toolOrientX
      b = self.toolOrientY
      cv2.arrowedLine(img_detection_region, (cmX,cmY), (a[0], a[1]), redColor, 2, 8, 0, 0.1)
      cv2.arrowedLine(img_detection_region, (cmX,cmY), (b[0], b[1]), greenColor, 2, 8, 0, 0.1)

      if not disable_servo:
        # Draw arrow from detected tool center of mass to center of the image
        # This arrow will also be used as the visual servoing command
        cv2.arrowedLine(img_detection_region, (cmX,cmY), (originX, originY), (0,255,255), 2, 8, 0, 0.1)

    if trocarDetected:
      for i in range(1, len(contoursGreen)):
      # draw ith contour
        cv2.drawContours(img_detection_region, contoursGreen, i, contour_color, 2, 8, hierarchy)

    # Draw Color Region Of Interest
    # cv2.polylines(cv_image, [colorROI], True, (0,255,0), thickness=3)

    # Draw Tool Region Of Interest
    cv2.polylines(cv_image, [toolROI], True, (255,255,255), thickness=2)

    # Draw grasp points and their triangle
    for point in reducedIntersectionPoints:
      cv2.circle(img_detection_region,(point[1],point[0]),7,(10,255,255),-1)
    grasp_triangle = np.flip(np.array(reducedIntersectionPoints)).reshape((-1,1,2))
    cv2.polylines(cv_image, [grasp_triangle], True, (0,255,255))

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
  rospy.init_node('opencv_image_view', anonymous=True)
  ic = Detection()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


help_string = """
Copmuter Vision and Visual servoing for KUKA-Barrett robot

rosrun vision visual_servo.py [OPTIONS]

OPTIONS:
  -h, --help
  -d, --disable-servo
  -e, --show-error-graph
"""
if __name__ == '__main__':
  if "--help" in sys.argv or "-h" in sys.argv:
    print(help_string)
  else:
    if "--disable-servo" in sys.argv or "-d" in sys.argv:
      disable_servo = True
    if "--show-error-graph" in sys.argv or "-e" in sys.argv:
      show_error_graph = True

    main(sys.argv)