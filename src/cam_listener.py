#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

x = []
y = []

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("ball_center",Point, queue_size = 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

  def callback(self,data):
    global x
    global y
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      cv2.waitKey(3)
    except CvBridgeError as e:
      print(e)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define range of red color in HSV
    lower_red = np.array([0,50,50])
    upper_red = np.array([4,255,255])

    # Threshold the HSV image to get only red colors
    mask = cv2.inRange(hsv, lower_red, upper_red)
      # Bitwise-AND mask and original image
    #res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
    blurred_mask = cv2.GaussianBlur(mask,(9,9),3,3)
    im2, contours, hierarchy = cv2.findContours(blurred_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        #find largest contour in mask, use to compute minEnCircle 
        c = max(contours, key = cv2.contourArea)
        ((x,y), radius) = cv2.minEnclosingCircle(c)
        #M = cv2.moments(c)
        cv2.circle(cv_image,(int(x),int(y)),10,255,-1)
        cv2.drawContours(cv_image,c,-1, (0,255,0),3)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(float(x),float(y),0.0)
      print([x,y])
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
