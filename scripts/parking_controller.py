#!/usr/bin/env python

import rospy
import math
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.dist_thresh = self.parking_distance + 0.5 # parameterize later
        self.angle_thresh = np.pi/4
        self.previous_angle = 0
        self.kd = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #pseudocode
        #if relative distance too close to car or extreme angle
            # execute reversal. Function of theta to determine which direction to do reversal in
        # else
            # use relative angle as steering angle plus some damping with derivative term to avoid overshoot
            # and oscillation
        
        theta = np.arctan2(self.relative_y, self.relative_x)
        if np.sqrt(self.relative_x**2 + self.relative_y**2) < self.dist_thresh or abs(theta) > angle_thresh:
            self.revesal(theta)
        else:
            self.drive_cmd.speed = abs(self.drive_cmd.speed)
            self.drive_cmd.steering_angle = theta + self.kd*(theta - self.previous_angle)
            self.previous_angle = theta

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def reversal(self, theta):
        self.drive_cmd.steering_angle(-theta)
        self.drive_cmd.speed = -abs(self.drive_cmd.speed)

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.y_error = self.relative_y
        error_msg.x_error = self.relative_x - self.parking_distance
        error_msg.distance_error = math.hypot(error_msg.x_error, error_msg.y_error)

        #################################
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
