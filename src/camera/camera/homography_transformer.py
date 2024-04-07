import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from visualization_msgs.msg import Marker
from visual_servoing.msg import ConeLocation, ConeLocationPixel
from geometry_msgs.msg import Point

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[137, 402],
                   [631, 276],
                   [116, 221],
                   [599, 184],
                   [112, 166]] # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[0.3, 0.1],
                    [0.5, -0.15],
                    [0.7, 0.2],
                    [0.9, -0.25],
                    [1.1, 0.3]] # dummy points
######################################################


class HomographyTransformerNode(Node):
    def __init__(self):
        super().__init__('homography_transformer')
        self.cone_px_sub = self.create_subscription(Point, '/block_px', self.cone_detection_callback, 10)

        self.cone_pub = self.create_publisher(Point, "/relative_cone", qos_profile=10)

        #Initialize data into a homography matrix
        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def cone_detection_callback(self, msg):
        #Extract information from message
        u = msg.x
        v = msg.y

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = Point()
        relative_xy_msg.x = x
        relative_xy_msg.y = y
        relative_xy_msg.z = 0.0


        self.cone_pub.publish(relative_xy_msg)


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

def main():
    rclpy.init()
    homography_transformer = HomographyTransformerNode()
    rclpy.spin(homography_transformer)

    # Destroy the node!
    homography_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()