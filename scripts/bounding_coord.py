#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan

# Example 2D and 3D points for initial calibration (replace with your own points)
points_2D = np.array([
    (155, 330),
    (210, 280),
    (320, 280),
    (415, 280),
], dtype="double")

points_3D = np.array([
    (-1.0732, -0.17268, 0),
    (-1.6316, -0.17515, 0),
    (-1.6426, 0.39314, 0),
    (-1.6522, 0.68096, 0)
], dtype="double")

# Define the camera matrix and distortion coefficients
cammat = np.array([[800, 0, 320],
                   [0, 800, 240],
                   [0, 0, 1]], dtype="double")

dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

# Initialize the CvBridge
bridge = CvBridge()

# Global variables to store the rotation and translation vectors
rvec = None
tvec = None

# Dummy bounding box (replace with actual input)
bounding_box = (150, 270, 350, 400)  # (x_min, y_min, x_max, y_max)

def solve_pnp():
    global rvec, tvec
    retval, rvec, tvec = cv2.solvePnP(points_3D, points_2D, cammat, dist_coeffs)
    if retval:
        rospy.loginfo(f"Rotation Vector: {rvec}")
        rospy.loginfo(f"Translation Vector: {tvec}")
    else:
        rospy.logerr("Could not solve PnP problem.")

def get_lidar_points_in_bbox(scan, bounding_box):
    global rvec, tvec
    if rvec is None or tvec is None:
        rospy.logwarn("PnP not solved yet")
        return []

    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    points_lidar = np.array([(r * np.cos(a), r * np.sin(a), 0) for r, a in zip(scan.ranges, angles) if scan.range_min < r < scan.range_max])

    if points_lidar.size == 0:
        return []

    points_img, _ = cv2.projectPoints(points_lidar, rvec, tvec, cammat, dist_coeffs)
    points_img = points_img.reshape(-1, 2)

    x_min, y_min, x_max, y_max = bounding_box
    lidar_points_in_bbox = []

    for (x, y), (x_lidar, y_lidar) in zip(points_img, points_lidar[:, :2]):
        if x_min <= x <= x_max and y_min <= y <= y_max:
            distance = np.sqrt(x_lidar**2 + y_lidar**2)
            angle = np.arctan2(y_lidar, x_lidar)
            lidar_points_in_bbox.append((x_lidar, y_lidar, distance, angle))

    return lidar_points_in_bbox

def camera_cb(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # Draw bounding box for visualization
    x_min, y_min, x_max, y_max = bounding_box
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    cv2.imshow("Image", frame)
    cv2.waitKey(1)

def lidar_cb(scan):
    try:
        img_msg = rospy.wait_for_message("/usb_cam/image_raw", Image, timeout=1.0)
        frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except (CvBridgeError, rospy.ROSException) as e:
        rospy.logerr("Error getting image: {0}".format(e))
        return

    # Draw bounding box for visualization
    x_min, y_min, x_max, y_max = bounding_box
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

    # Get LiDAR points in bounding box
    lidar_points = get_lidar_points_in_bbox(scan, bounding_box)

    # Print distances and angles of points in bounding box
    for x, y, distance, angle in lidar_points:
        rospy.loginfo(f"Point in bbox - Distance: {distance:.2f} m, Angle: {np.degrees(angle):.2f} degrees")

    # Display the image
    cv2.imshow("Image with LiDAR", frame)
    cv2.waitKey(1)

def main(args=None):
    rospy.init_node('calib_node')
    solve_pnp()
    
    # Subscribe to the camera and LiDAR topics
    rospy.Subscriber("/usb_cam/image_raw", Image, camera_cb, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_cb, queue_size=1)

    # Spin to keep the script running
    rospy.spin()

    # Destroy all OpenCV windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
