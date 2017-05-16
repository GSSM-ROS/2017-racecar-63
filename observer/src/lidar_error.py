#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt


class Observer():
    def __init__(self, goal, slice_indeces):
        rospy.init_node('observer', anonymous=False)
        rospy.Subscriber('/scan', LaserScan, self.display)

        self.goal = goal
        self.slice_indeces = slice_indeces
        self.errors = []

        rospy.spin()

    def display(self, scan_data):
        dist = min(scan_data[slice_indeces[0], slice_indeces[1]])
        error = goal - dist
        self.errors.append(error)
        if len(self.errors) > 1000:
            self.errors.pop(0)
        plt.plot(self.errors)
        plt.show()

Observer(0.7, (450,630))
