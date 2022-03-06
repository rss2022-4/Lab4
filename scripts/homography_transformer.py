#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from visual_servoing.msg import ConeLocation, ConeLocationPixel

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

### CALIBRATION TRIAL 1 ###

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

# ######################################################
# ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# PTS_IMAGE_PLANE = [[406.0, 276.0], # airpod_left_bottom_corner
#                    [309.0, 240.0], # green_post-its_left_bottom_corner
#                    [317.0, 295.0], # contact_lenses_right_bottom_corner
#                    [234.0, 252.0]] # matcha_pocky_box_right_bottom_corner
# ######################################################

# # PTS_GROUND_PLANE units are in inches
# # car looks along positive x axis with positive y axis to left

# ######################################################
# ## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# # 1. measured with paper ruler, 2. measured with iPhone Measure App (Less precise)
# PTS_GROUND_PLANE = [[14.375, -3.625], # [14.375, 3.000] 
#                     [23.875, 1.3125], # [24.875, -1.500]
#                     [20.9375, 2.625], # [21.750, -2.000]
#                     [20.625, 8.625]] # [20.750, -8.500]
# ######################################################


### CALIBRATION TRIAL 2 ###

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[436.0, 305.0], # blue 
                   #[230.0, 274.0], # purple
                   [484.0, 257.0], # red/orange
                   [217.0, 248.0], # green
                   [336.0, 237.0]] # yellow
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[10.70, -4.5625], # blue 
                   #[16.750, 7.5000], # purple
                   [20.125, -11.75], # red/orange
                   [23.50, 10.3125], # green
                   [27.25, -1.4375]] # yellow
######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer:
    def __init__(self):
        self.cone_px_sub = rospy.Subscriber("/relative_cone_px", ConeLocationPixel, self.cone_detection_callback)
        self.cone_pub = rospy.Publisher("/relative_cone", ConeLocation, queue_size=10)

        self.marker_pub = rospy.Publisher("/cone_marker", Marker, queue_size=1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def cone_detection_callback(self, msg):
        #Extract information from message
        u = msg.u
        v = msg.v

        #Call to main function
        x, y = self.transformUvToXy(u, v)

        #Publish relative xy position of object in real world
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y

        self.cone_pub.publish(relative_xy_msg)
        self.draw_marker(x,y,'map',0)

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

    def draw_marker(self, cone_x, cone_y, message_frame, id):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.id = id
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('homography_transformer')
    homography_transformer = HomographyTransformer()
    rospy.spin()

    # ### TESTING CALIBRATION ###
    # for pixel_point in PTS_IMAGE_PLANE:
    #     x,y = homography_transformer.transformUvToXy(pixel_point[0],pixel_point[1])
    #     print(x/METERS_PER_INCH, y/METERS_PER_INCH)
    
    # # CALIBRATION TRIAL 1
    # # x,y = homography_transformer.transformUvToXy(303.0,324.0)
    # # print(x/METERS_PER_INCH, y/METERS_PER_INCH)
    # # correct xy values = [x = 8.5, y = 2.5625]

    # # CALIBRATION TRIAL 2
    # x,y = homography_transformer.transformUvToXy(230.0,274.0)
    # print(x/METERS_PER_INCH,y/METERS_PER_INCH)
    
    # plot markers on RVIZ to check homography matrix
    # while not rospy.is_shutdown():
    #     for i, pixel_point in enumerate([[436.0, 305.0], # blue 
    #                                     [230.0, 274.0], # purple
    #                                     [484.0, 257.0], # red/orange
    #                                     [217.0, 248.0], # green
    #                                     [336.0, 237.0]]): # yellow:
    #         x,y = homography_transformer.transformUvToXy(pixel_point[0],pixel_point[1])
    #         homography_transformer.draw_marker(x*5,y*5,'map', i) # scaled by 5 for better display


