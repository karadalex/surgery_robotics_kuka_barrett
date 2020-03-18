import cv2


"""
A1-------A2
|       |
|  cm   |
|       |
A3-------A4
"""
def getRectangularNeighborhood(centerPixel, distanceFromCenter, cvImage, draw = False):
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