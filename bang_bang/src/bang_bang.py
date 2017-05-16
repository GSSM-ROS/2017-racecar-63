#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class BangBanger():
    def __init__(self):
        rospy.init_node("bangbang", anonymous=False)
        rospy.Subscriber("/scan", LaserScan, callback=self.read_lasers)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=6)
        rospy.on_shutdown(self.die)
        self.MAX_SPEED = 2
        self.goal = 0.5
        self.straightness = 0.1
        rospy.spin()

    def read_lasers(self, data):
        scan_data = data.ranges
        #min_dist = min(scan_data[50:540]) #right follow
        min_dist = min(scan_data[540:1030]) #left follow
        error = self.goal - min_dist

        steering = 0
        if abs(error) > self.straightness:
            #steering = 1 if error > 0 else -1 #right follow
            steering = -1 if error > 0 else 1 #left follow

        self.msg = AckermannDriveStamped()
        self.msg.drive.speed = self.MAX_SPEED
        self.msg.drive.steering_angle = steering
	
        if min(scan_data[400:680]) < 0.5:
            #self.msg.drive.steering_angle = -steering
            self.msg.drive.speed = -0.2
	
	self.pub.publish(self.msg)

    def die(self):
        print("BangBanger down")

BangBanger()
