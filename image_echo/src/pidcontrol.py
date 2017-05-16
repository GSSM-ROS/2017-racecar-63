#! /usr/bin/env python

from geometry_msgs.msg import Point #this is the data type given to this program to be processed into steering directions
import rospy #this is used to make publishers and subscribers for communication between programs
from ackermann_msgs.msg import AckermannDriveStamped #this is what is published to give steering directions to the drive file
from std_msgs.msg import Float64MultiArray #for pid publishing


class PID(): #declares the class for the PID controller
    def callback(self, msg): #this is a callback
        self.send = AckermannDriveStamped() #declares self.send to be an AckermannDriveStamped object
        u = self.pid((self.goal)-(msg.x))         #gets pid calculation self.goal-msg.x=error
        self.send.drive.steering_angle = u #sets the drive steering angle value of self.send equal to the u created by the pid controller
        self.pub.publish(self.send) #publishes the self.send for use by the robot

    def __init__(self): #initiates self
        rospy.init_node("pidsteer", anonymous=False) #creates a pidsteer node for publishing and subscribing
        self.pub=rospy.Publisher("/contour_steering", AckermannDriveStamped, queue_size=1) #creates a publisher to /contour_steering which is what the other program will use for a drive controller.
        self.pidpub = rospy.Publisher("/contour_steering_pid", Float64MultiArray, queue_size=1) #for publishing pid to debug
        rospy.Subscriber("/contour_xyz", Point, self.callback) #creates a subscriber to /contour_xyz which gives x and y values of the center of the contour.
        rospy.Subscriber("/contour_speed", AckermannDriveStamped, self.speed_callback) #gets speed from other publisher

        self.goal=1280/2 #this is the goal. there are 1280 pixels on the x axis, therefore 1280/2 is assumed to be the middle and goal.
        self.Kp=0.00055 #this declares the Kp value
        self.Ki=0 #this declares the Ki value
        self.Kd=0.0000001
 #this declares the Kd value
        self.errors=[] #this creates a list for errors to be in
        self.period=40 #this is the limit on the length of the list
        self.dt=0.00005 #declares a dt value
#        print "HI" was used in testing to ensure that the program got to this point on run.

        self.speed = 0 #speed of robot (to scale steering)

        rospy.spin() #makes the program hang until stopped

    def speed_callback(self, msg):
        '''
        Gets speed of racecar to scale the steering constant
        '''
        self.speed = msg.drive.speed

    def pid(self, error):                                 #creating a pid controller                   
        '''
        :param self: class referential variable
        :param error: distance from desired goal
        '''
        self.errors.append(error) #adds most recent error to the list of errors
        if len(self.errors) > self.period: #removes the oldest error once the length exceeds the maximum length
            self.errors.pop(0)
        p = self.Kp * error #calculation for p-value
        if len(self.errors) > 1: #calculation for d-value so long as it is after the first measurement
            d = self.Kd * (error - self.errors[-2]) / self.dt  
        else:
            d = 0 #d=0 on the first measurement
        i = self.Ki * sum(self.errors)/len(self.errors)*self.dt #calculation for i                        
        u = (p + i + d) / (abs(self.speed) + 1) #calculation for u

        self.pid_msg = Float64MultiArray() # does stuff
        self.pid_msg.data = [p, d, i] # does things 
        self.pidpub.publish(self.pid_msg) # does stuff and things whilst debugging

        return u #gives u to whatever asked for it
PID() #calls the program
