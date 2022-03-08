#!/usr/bin/env python

from re import M
import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """

    def __init__(self):
        # toggle line follower vs cone parker
        # TODO: not sure what this boolean is supposed to do?
        #  as far as I can tell, the modifications for line following take place in parking_controller, not here
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher(
            "/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher(
            "/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber(
            "/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge()  # Converts between ROS images and OpenCV Images

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")

        # code could be refactored to remove the None template
        bbox = cd_color_segmentation(image, None)
        tleft = bbox[0]  # top left of bounding box
        bright = bbox[1]  # bottom right of bounding box
        # calculates middle pixel across x-axis
        x_avg = (tleft[0] + bright[0]) // 2
        # the y pixel coordinate of the bottom of the bounding box
        y_bottom = bright[1]

        cone_loc = ConeLocationPixel()
        cone_loc.u = x_avg
        cone_loc.v = y_bottom
        self.cone_pub.publish(cone_loc)
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
