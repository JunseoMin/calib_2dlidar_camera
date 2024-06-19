#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

# Define 2D and 3D points for initial calibration (example points)
points_2D = np.array([
    (442, 444),
    (326, 329),
    (450, 353),
    (372, 385),
], dtype="double")

points_3D = np.array([
    (0.513, -0.011, 0),
    (0.865, -0.369, 0),
    (0.678, -0.172, 0),
    (0.226, -0.0165, 0)
], dtype="double")

# Define the camera matrix and distortion coefficients
# Camera Params
cammat = np.array([[382.225341796875, 0.0, 319.6082763671875],
                   [0.0, 381.72235107421875, 248.27783203125],
                   [0, 0, 1]], dtype="double")
dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Initialize the CvBridge
bridge = CvBridge()

# Global variables to store the rotation and translation vectors
rvec = None
tvec = None

def solve_pnp():
    global rvec, tvec
    retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cammat, dist_coeffs)
    
    if retval:
        rospy.loginfo(f"Rotation Vector: {rvec}")
        rospy.loginfo(f"Translation Vector: {tvec}")
    else:
        rospy.logerr("Could not solve PnP problem.")

def project_lidar_to_image(scan, image):
    global rvec, tvec
    rospy.logwarn("Projection start!!")

    if rvec is None or tvec is None:
        rospy.logwarn("PnP not solved yet")
        return
    rospy.logwarn("PnP solved")

    # Convert LaserScan to 2D points in LiDAR frame
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    points_lidar = np.array([(r * np.cos(a), r * np.sin(a), 0) for r, a in zip(scan.ranges, angles) if r < scan.range_max and r > scan.range_min])

    rospy.logwarn("angles, points set")

    # Project LiDAR points onto the image
    points_img, _ = cv2.projectPoints(points_lidar, rvec, tvec, cammat, dist_coeffs)
    points_img = points_img.reshape(-1, 2)

    # Create a copy of the image to make it writable
    writable_image = image.copy()
    rospy.logwarn("img copied")


    for point in points_img:
        # rospy.logwarn("points set writed {},{}".format(point[0],point[1]))
        
        x, y = int(point[0]), int(point[1])
        cv2.circle(writable_image, (x, y), 3, (0, 0, 255), -1)
    rospy.logwarn("setted")

    cv2.imshow("Image with LiDAR", writable_image)
    cv2.waitKey(1)

def camera_cb(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Display the image
    # cv2.imshow("Image", frame)
    # cv2.waitKey(1)

def lidar_cb(scan):
    try:
        img_msg = rospy.wait_for_message("/usb_cam/image_raw", Image, timeout=1.0)
        frame = bridge.imgmsg_to_cv2(img_msg)
    except (CvBridgeError, rospy.ROSException) as e:
        rospy.logerr("Error getting image: {0}".format(e))
        return

    project_lidar_to_image(scan, frame)



def main(args=None):
    rospy.init_node('calib_node')
    solve_pnp()
    
    # Subscribe to the camera and LiDAR topics
    rospy.Subscriber("/usb_cam/image_raw", Image, camera_cb, queue_size=10)
    rospy.Subscriber("/scan", LaserScan, lidar_cb, queue_size=10)

    # Spin to keep the script running
    rospy.spin()

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
