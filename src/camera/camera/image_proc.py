import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Point

from enum import Enum

# Min and max HSV thresholds
GREEN_THRESHOLD = ([40,15,15], [80,255,255])
RED_THRESHOLD_1 = ([0,100,100], [10, 255, 255])
RED_THRESHOLD_2 = ([170,100,100], [180,255,255])
BLUE_THRESHOLD = ([100,100,0],[140,255,255])

class Color(Enum):
    RED = 1
    GREEN = 2


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('image_proc')
        self.color = Color.RED
        self.image_sub = self.create_subscription(Image, '/v4l2_camera/image_raw', self.listener_callback, 10)
        self.image_sub
        self.block_color_pub = self.create_publisher(String, '/block_color', 10)
        self.block_px_pub = self.create_publisher(Point, '/block_px', 10)
        self.block_seen_pub = self.create_publisher(Bool, '/block_seen', 10)
        self.br = CvBridge()
        self.i = 0

    def listener_callback(self, msg):
        # Capture a frame from the webcam 
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # OpenCV needs bounds as numpy arrays

        lower_bound_g = np.array(GREEN_THRESHOLD[0])
        upper_bound_g = np.array(GREEN_THRESHOLD[1])

        lower_bound_r1 = np.array(RED_THRESHOLD_1[0])
        upper_bound_r1 = np.array(RED_THRESHOLD_1[1])

        lower_bound_r2 = np.array(RED_THRESHOLD_2[0])
        upper_bound_r2 = np.array(RED_THRESHOLD_2[1])

        lower_bound_b = np.array(BLUE_THRESHOLD[0])
        upper_bound_b = np.array(BLUE_THRESHOLD[1])


        # Threshold the HSV image to get only desired color
        # Mask contains a white on black image, where white pixels
        # represent that a value was within our green threshold.
        mask_b = cv2.inRange(hsv, lower_bound_b, upper_bound_b)

        mask_g = cv2.inRange(hsv, lower_bound_g, upper_bound_g)

        mask_r1 = cv2.inRange(hsv, lower_bound_r1, upper_bound_r1)
        mask_r2 = cv2.inRange(hsv, lower_bound_r2, upper_bound_r2)
        mask_r = mask_r1 + mask_r2

        # Find contours (distinct edges between two colors) in mask using OpenCV builtin
        # This function returns 2 values, but we only care about the first

        # Note: In some OpenCV versions this function will return 3 values, in which
        # case the second is the contours value. If you have one of those versions of
        # OpenCV, you will get an error about "unpacking values" from this line, which you
        # can fix by adding a throwaway variable before contours
        contours_r, _ = cv2.findContours(mask_r, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_g, _ = cv2.findContours(mask_g, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_b, _ = cv2.findContours(mask_b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Fine blue line if it exists
        blue_line=float('-inf')

        if len(contours_b) != 0:
            c_b= max(contours_b, key = cv2.contourArea)
            i,j,k,l = cv2.boundingRect(c_b)
            if k>0.25*frame.shape[1]:
                blue_line=j

        relevant=[]
        color=[]

        for cont in contours_r:
            a,b,c,d = cv2.boundingRect(cont)
            if (b+d)>blue_line and c*d>frame.shape[0]*frame.shape[1]*.0025:
                relevant.append(cont)
                color.append("red")

        for cont in contours_g:
            a,b,c,d = cv2.boundingRect(cont)
            if (b+d)>blue_line and c*d>frame.shape[0]*frame.shape[1]*.0025:
                relevant.append(cont)
                color.append("green")

        block_seen_msg = Bool()
        block_seen_msg.data = False
        largest_contour = None
        largest = 0
        idx = 0
        block_color_msg = String()

        if len(relevant) != 0:
            block_seen_msg.data = True
            for i in range(len(relevant)):
                size = cv2.contourArea(relevant[i])
                if size > largest:
                    largest_contour = relevant[i]
                    largest = size
                    idx = i

            block_color_msg.data = color[idx]


        if block_seen_msg.data:
            a,b,c,d = cv2.boundingRect(largest_contour)
            block_px_msg = Point()
            block_px_msg.x = float(a+(c/2))
            block_px_msg.y = float(b+d)
            block_px_msg.z = 0.0

            self.block_px_pub.publish(block_px_msg)
            self.block_color_pub.publish(block_color_msg)


        self.block_seen_pub.publish(block_seen_msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()