#! /usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class Driver():
    def __init__(self):
        '''
        Constructs node for drive control from contour detection controller
        '''
        rospy.init_node('driver', anonymous=False)                              #comment to appease Brennan
        rospy.Subscriber('/contour_speed', AckermannDriveStamped, self.speed_callback)#"Hey ervey body and welcom eback to my Minecraft-" subs to the speed
        rospy.Subscriber('/contour_steering', AckermannDriveStamped, self.steering_callback)#"Go ahead a SMASH that Like button-" subs to steering
        rospy.Subscriber('/scan', LaserScan, self.publish)                      #"NEW free HD Cheat Minecraft Diamonds $Cash-" subs to the lidar scan
        self.pub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1) # creates a meesage for the speed

        self.speed = 0              #message contents
        self.steering = 0
        self.unsafe_dist = 0.6

        rospy.spin()                #You spin me right round baby right round like a record baby round rounf right round

    def speed_callback(self, msg):  #specific callback for speed
        '''
        Updates speed of drive controller
        :param msg: AckermannDriveStamped message containing speed
        '''
        self.speed = msg.drive.speed

    def steering_callback(self, msg):
        '''
        Updates steering_angle of drive controller
        :param msg: AckermannDriveStamped message containing steering_angle
        '''
        self.steering = msg.drive.steering_angle

    def publish(self, msg):
        '''
        Uses LaserScan to make sure distance is safe then publishes
        :param msg: LaserScan message from LiDAR
        '''
        if min(msg.ranges[450:630]) < self.unsafe_dist: # Its the safety Dance
            #safety controller
            self.speed = -0.2
        self.send = AckermannDriveStamped()

        self.send.drive.speed = self.speed
        self.send.drive.steering_angle = self.steering #publishes the entire message for drive
        self.pub.publish(self.send)



Driver()
