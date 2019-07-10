#!/usr/bin/env python
import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    def __init__(self):
        # Initialize your publishers and
        # subscribers
        self.data = None    
        self.angle = 0
        self.diffAngle = 1
        self.constAngle = 1 
        self.cmd = AckermannDriveStamped()
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
    def scan(self, data):
    #stores the lidar data so you can work with it
        self.data = data
    #calls function that controls driving
        self.drive()
    
    def drive(self):
    #controls driving
    #gets the angle required
        self.angle = self.find_wall()
    #sets speed and driving angle
        self.cmd.drive.speed = self.VELOCITY
        self.cmd.drive.steering_angle = self.angle
        #publishes the command
        self.drive_pub.publish(self.cmd)

    def find_wall(self):
    # if lidar data has not been received, do nothing
        e1 = self.data.ranges[self.data.ranges.len()-1] - self.DESIRED_DISTANCE
        e2 = self.data.ranges[self.data.ranges.len()/4*3-1] - self.DESIRED_DISTANCE
        if e1 > 0 and e2 > 0:
            self.diffAngle = self.constAngle
            tempAngle = self.diffAngle
        elif e1 > 0 and e2 < 0:
            self.diffAngle = -self.diffAngle/2
            tempAngle = -self.diffAngle
        elif e1 < 0 and e2 > 0:
            self.diffAngle = self.diffAngle/2
            tempAngle = self.diffAngle
        elif e1 < 0 and e2 < 0:
            self.diffAngle = -self.constAngle/2
            tempAngle = -self.diffAngle
        if self.data.ranges[self.data.ranges.len()/2-1] < 1.41 * self.DESIRED_DISTANCE:
            tempAngle = -self.diffAngle
        return tempAngle


"""Lidar data is now stored in self.data, which can be accessed
using self.data.ranges (in simulation, returns an array).
Lidar data at an index is the distance to the nearest detected object
self.data.ranges[0] gives the leftmost lidar point
self.data.ranges[100] gives the rightmost lidar point
self.data.ranges[50] gives the forward lidar point
"""
#returns the output of your alg, the new angle to drive in

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
rospy.spin()
