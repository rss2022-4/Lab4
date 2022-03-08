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
        self.total_error_pub = rospy.Publisher("/total_parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.dist_thresh = self.parking_distance + 0 # parameterize later
        self.angle_thresh = np.pi/4
        self.parking_epsilon = 0.1
        self.angle_epsilon = 5*np.pi/180.
        self.previous_angle = 0
        self.kd = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = self.initialize_drive_data()
        
        theta = np.arctan2(self.relative_y, self.relative_x)
        distance = np.sqrt(self.relative_x**2 + self.relative_y**2)
        #print('theta', theta)
        #print('distance', distance)
        #print('x', self.relative_x)
        #print('y', self.relative_y)
        # Within tolerable desired distance and angle with respect to the cone
        if abs(distance - self.dist_thresh) < self.parking_epsilon and abs(theta) < self.angle_epsilon:
            #print('in stopping condition')  
            drive_cmd = self.stop(drive_cmd)
        # Either too close or at an extreme angle. Correct by executing reversal
        elif distance < self.dist_thresh or abs(theta) > self.angle_thresh:
            #print('extreme angle or extreme distance')
            drive_cmd = self.reversal(theta, drive_cmd)
        # Within reasonable area. Use controller
        else:
            #print('in nominal condition')
            drive_cmd = self.drive(theta, drive_cmd)

        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def stop(self, drive_cmd):
        drive_cmd.drive.speed = 0
        drive_cmd.drive.steering_angle = 0
        return drive_cmd
    
    def reversal(self, theta, drive_cmd):
        drive_cmd.drive.steering_angle = -theta
        drive_cmd.drive.speed = -abs(drive_cmd.drive.speed)
        return drive_cmd
    
    def drive(self, theta, drive_cmd):
        drive_cmd.drive.speed = abs(drive_cmd.drive.speed)
        drive_cmd.drive.steering_angle = theta + self.kd*(theta - self.previous_angle)
        self.previous_steering_angle = theta
        return drive_cmd

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
        self.total_error_pub.publish(error_msg)

    def initialize_drive_data(self):
        drive_data = AckermannDriveStamped()
        drive_data.drive.speed = 0.5
        return drive_data

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
