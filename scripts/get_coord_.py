#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import LaserScan,Image

bridge = CvBridge()

def mouse_callback(event, x, y, flags, param):
  if event == cv2.EVENT_LBUTTONDOWN:
    print(f"cmaeran coord: ({x}, {y})")

def camera_cb(msg):
  in_img = bridge.imgmsg_to_cv2(msg)
  cv2.namedWindow("image")
  cv2.imshow("image", in_img)

  cv2.setMouseCallback("image", mouse_callback)
  cv2.waitKey(0)

def main(args=None):
  rospy.init_node('calib_node')
  cam_subs = rospy.Subscriber("/usb_cam/image_raw", Image, camera_cb,queue_size=100)
  rospy.spin()


if __name__ == '__main__':
  main()