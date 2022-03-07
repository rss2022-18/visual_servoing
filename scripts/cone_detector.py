#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point  # geometry_msgs not in CMake file
from visual_servoing.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
# from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """

    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

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

    def image_print(self, img):
        """
        Helper function to print out images, for debugging. Pass them in as a list.
        Press any key to continue.
        """
        cv2.imshow("image", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def cd_color_segmentation(self, img, template):
        """
        Implement the cone detection using color segmentation algorithm
        Input:
            img: np.3darray; the input image with a cone to be detected. BGR.
            template_file_path; Not required, but can optionally be used to automate setting hue filter values.
        Return:
            bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                    (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
        """
        ########## YOUR CODE STARTS HERE ##########
        # ORANGE_THRESHOLD = ([5,50,50], [15,255,255])
        #
        #
        #
        #   #HSV (Hue, Saturation, Value)   For Orange color
        ORANGE_THRESHOLD = ([5, 50, 50], [15, 255, 255])
        bounding_box = ((0, 0), (0, 0))

        # frame = cv2.imread(img)
        frame = img
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # OpenCV needs bounds as numpy arrays
        lower_bound = np.array(ORANGE_THRESHOLD[0])
        upper_bound = np.array(ORANGE_THRESHOLD[1])

        # Threshold the HSV mage to get only green color
        # TODO: why does the above comment state green?
        # Mask contains a white on black image where white pixels represent that a value was within our orange Threshold.

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        bounding_box = ((0, 0), (0, 0))

        _, contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) != 0:
            print(contours)
            c = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
            bounding_box = ((x, y), (x + w, y + h))

        ########### YOUR CODE ENDS HERE ###########

        # Return bounding box
        return bounding_box


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
