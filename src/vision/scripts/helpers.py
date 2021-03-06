import cv2
import numpy as np
import math
import geometry


def whitenBoundary(image, img_detection_region):
  """make boundary of image white so that contours will be always closed loops within the region

  Args:
      image ([type]): [description]
      img_detection_region ([type]): [description]

  Returns:
      [type]: [description]
  """
  image[0:img_detection_region.shape[0], 0].fill(255)
  image[0:img_detection_region.shape[0], img_detection_region.shape[1]-1].fill(255)
  image[0, 0:img_detection_region.shape[1]].fill(255)
  image[img_detection_region.shape[0]-1, 0:img_detection_region.shape[1]].fill(255)

  return image


def getRectangularNeighborhood(centerPixel, distanceFromCenter, cvImage, draw = False):
  """
  A1-------A2
   |       |
   |  cm   |
   |       |
  A3-------A4

  Args:
      centerPixel ([type]): [description]
      distanceFromCenter ([type]): [description]
      cvImage ([type]): [description]
      draw (bool, optional): [description]. Defaults to False.

  Returns:
      [type]: [description]
  """
  x,y = centerPixel[0], centerPixel[1]
  # Only A1 and A4 are needed to define the rectangle
  A1 = [x-distanceFromCenter, y-distanceFromCenter]
  A4 = [x+distanceFromCenter, y+distanceFromCenter]

  (rows,cols,_) = cvImage.shape

  # Check if neighborhood escapes image
  if A1[0] < 0:
    A1[0] = 0
  if A4[1] < 0:
    A4[0] = 0
  if A1[0] > cols:
    A1[0] = cols
  if A4[1] > rows:
    A4[0] = rows

  if draw:
    cv2.rectangle(
      cvImage,
      (A1[0], A1[1]),  # top left corner
      (A4[0], A4[1]),  # bottom right corner
      (255,0,0),
      2 # 2px thickness
    )

  return cvImage[A1[1]:A4[1], A1[0]:A4[0]]


def polygonMeanDistanceFromCenter(polygon, cm):
  """[summary]

  Args:
      polygon ([type]): [description]
      cm ([type]): [description]

  Returns:
      [type]: [description]
  """
  sumD = 0
  for point in polygon:
    distance = math.sqrt((point[0][0] - cm[0])**2 + (point[0][1] - cm[1])**2)
    sumD += distance
  avgD = sumD/len(polygon)
  return avgD


def checkApproximateColorEq(px, color, r):
  # Check if blue channel is approximately the same
  if color[0] - r <= px[0] and px[0] <= color[0] + r:
    # Check if green channel is approximately the same
    if color[1] - r <= px[1] and px[1] <= color[1] + r:
      # Check if red channel is approximately the same
      if color[2] - r <= px[2] and px[2] <= color[2] + r:
        return True
  return False


def hasPolygonCenterOfColor(img, hull, color, r=5):
  """[summary]

  Args:
      img ([type]): OpenCV image
      hull ([type]): Convex Hull
      color ([type]): Color in BGR format
      r (int, optional): [description]. Defaults to 10.

  Returns:
      [type]: [description]
  """
  cm = geometry.centerOfMass(hull)
  px = img[cm[1], cm[0]]
  if checkApproximateColorEq(px, color, r):
    return True

  # Check some neighbor pixels as well
  for i in range(1, 20):  
    try:
      px = img[cm[1]+i, cm[0]+i]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1]+i, cm[0]-i]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1]-i, cm[0]+i]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1]-i, cm[0]-i]
      if checkApproximateColorEq(px, color, r):
        return True

      px = img[cm[1]+i, cm[0]]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1]-i, cm[0]]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1], cm[0]+i]
      if checkApproximateColorEq(px, color, r):
        return True
      px = img[cm[1], cm[0]-i]
      if checkApproximateColorEq(px, color, r):
        return True
    except IndexError:
      continue
  
  return False


def isPolygonInROI(polygon, roi):
  """Check if Every Point of Polygon is inside ROI
  See also https://docs.opencv.org/3.1.0/d5/d45/tutorial_py_contours_more_functions.html

  Args:
      polygon ([type]): [description]
      roi ([type]): [description]

  Returns:
      [type]: [description]
  """
  for point in polygon:
    point = tuple(np.reshape(point, 2))
    if cv2.pointPolygonTest(roi, point, False) == -1:
      return False
  
  return True