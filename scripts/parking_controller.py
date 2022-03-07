#!/usr/bin/env python

import rospy
import numpy as np
from cmath import cos, sin, sqrt
from math import atan2
from turtle import distance

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

        self.desired_velocity = 1.0 #[m/s]
        self.L = 0.35 #[m]
        self.lookahead = 0.70 # [m], variable
        self.parking_distance = .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################

        #for now, drive directly to the cone and stay there. 
        # we implement the pure pursuit controller
        # port over the pure pursuit from before. 

        #distance 
        distance_from_cone = sqrt(self.relative_x**2 + self.relative_y**2).real
        angle_to_cone      = atan2(self.relative_y, self.relative_x)
        numerator = 2*self.L*sin(angle_to_cone).real
        denominator = self.lookahead.real

        # pure pursuit nominal case
        delta = (atan2(numerator, denominator)).real # wow, this one worked really well. 
        
        # velocity scaling, according to angle. 
        vel = abs(self.desired_velocity*cos(angle_to_cone).real)

        if abs(vel) < 0.25: #prevent sticking and not moving. 
            vel = 0.25
        

        if distance_from_cone < self.parking_distance*2 : #start slowing down
            slowdown_normalized = (distance_from_cone-self.parking_distance)/self.parking_distance
            vel = vel*(slowdown_normalized)

            if distance_from_cone >= self.parking_distance-0.05 and distance_from_cone <= self.parking_distance +0.05:
                vel = 0
            elif distance_from_cone < self.parking_distance:
                vel = -vel

        # want to face the cone, not just back into it. 
        if abs(angle_to_cone) > np.pi*0.5:
            #just turn in place
            if angle_to_cone > np.pi*0.5:
                delta = np.pi*0.25
                print('spinning right')

            elif angle_to_cone < -np.pi*0.5:
                delta = -np.pi*0.25
                print('spinning left ')

        drive_cmd.drive.steering_angle = delta
        drive_cmd.drive.speed          = vel
        
        #maybe baselink and everything else is already implemented?


        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        #################################
        error_msg.x_error = float(self.relative_x)
        error_msg.y_error = float(self.relative_y)
        error_msg.theta_error = atan2(self.relative_y, self.relative_x)
        #print(sqrt((self.relative_x)**2 + (self.relative_y)**2).real)
        error_msg.distance_error = float((sqrt((self.relative_x)**2 + (self.relative_y)**2)).real)
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
