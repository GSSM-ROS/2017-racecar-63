#! /usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class Safety():
    def __init__(self):
        rospy.init_node("safety", anonymous=False)
        rospy.Subscriber("/scan", LaserScan, callback=self.read_lasers)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=6)
        rospy.on_shutdown(self.die)
        self.unsafe = 0.3
        rospy.spin()

    def read_lasers(self, data):
        scan_data = data.ranges
        min_dist = min(scan_data[400:680])

        if min_dist < self.unsafe:
            self.msg = AckermannDriveStamped()
            self.msg.drive.speed = -0.2
            self.msg.drive.steering_angle = 0
            self.pub.publish(self.msg)

    def die(self):
        print("Safety man down")

Safety()
