#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class image_saver:
 
  def __init__(self):
    self.image_pub = rospy.Publisher("/processed_image",Image,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/copter/cam/copterCam",Image,self.callback,queue_size=1)
 
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print("Error")
 
    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)
 
    cv2.imwrite("outputImg.jpg", cv_image)
    # loadedImg=cv2.imread("outputImg.jpg")
    # cv2.imshow("Image window", loadedImg)
    cv2.waitKey(3)
 
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print("Error")
 
def main(args):
  rospy.init_node('image_saver', anonymous=True)
  ic = image_saver()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  # cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)