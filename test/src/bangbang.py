#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy as np
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs

class Follower():
    
    '''
    ******************************************************************************************
    *                                  Begin Driving stuffs                                  *
    ******************************************************************************************
    '''
        
    '''
    callback for the laser subscriber
    '''
    def laserCall(self, msg):

	L = msg.ranges[540:1000]

	minPoint = min(L)

	error = .5-minPoint

	if error > .1:
		angle= -1
	elif error < -.1:
		angle = 1
	else:
		angle=0

	speed = -.3 if min(msg.ranges[400:680])<.3 else 1
        #create the new message
        drive_cmd = AckermannDriveStamped()
        
        
        #Assign speed and angle to the message
        drive_cmd.drive.steering_angle=angle
	drive_cmd.drive.speed = speed

        self.drive.publish(drive_cmd) # post this message
      
    
    '''
    *************************************************************************************************
    *                      Constructor and initialization of instance variables                     *
    *************************************************************************************************
    '''
    def __init__(self):
        '''
        Node setup and start
        '''
        rospy.init_node('grand_prix', anonymous=False)
        self.drive = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=5)
        rospy.Subscriber('scan', LaserScan, self.laserCall)
        
        '''
        Leave the robot going until roscore dies, then set speed to 0
        '''
        rospy.spin()
        self.drive.publish(AckermannDriveStamped())
        
if __name__=="__main__":
    Follower()

