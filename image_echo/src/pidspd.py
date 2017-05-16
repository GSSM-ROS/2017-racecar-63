#! /usr/bin/env python

from geometry_msgs.msg import Point #this is the data type given to this program to be processed into steering directions
import rospy #this is used to make publishers and subscribers for communication between programs
from ackermann_msgs.msg import AckermannDriveStamped #this is what is published to give steering directions to the drive file
from std_msgs.msg import Float64MultiArray #for publishing pid to debug


class PID(): #declares the class for the PID controller
    def callback(self, msg): #this is a callback
        if msg.z == float("nan"): print("msg.z is nan") #TODO remove
        self.send = AckermannDriveStamped() #declares self.send to be an AckermannDriveStamped object
        u=self.pid((msg.z-self.goal)) #gets pid calculation self.goal-msg.z=error
        self.send.drive.speed=u #sets the drive speed value of self.send equal to the u created by the pid controller
        self.pub.publish(self.send)#publishes the self.send for use by the robot


    def __init__(self): # initiates self
        rospy.init_node("pidspd", anonymous=False)#creates a pidspd node for publishing and subscribing
        self.pub=rospy.Publisher("/contour_speed", AckermannDriveStamped, queue_size=2)#creates a publisher to /contour_speed for controlling speed
        rospy.Subscriber("/contour_xyz", Point, self.callback)#creates a subscriber to /contour_xyz for receiving the depth
        self.pidpub = rospy.Publisher("/contour_speed_pid", Float64MultiArray, queue_size=1)#publisher used for debugging

        self.goal=1 #this is the goal distance in meters
        self.Kp=1 #declares the Kp value
        self.Ki=3 # declares the Ki value
        self.Kd=0.001 # declares the Kd value
        self.errors=[] #initiates a list for errors to be located in
        self.period=40 # sets a max error list length of 40
        self.dt=0.00005 # declares a dt value

        rospy.spin() # keeps the program running waiting for a callback to be triggered

    def pid(self, error): # creating a pid controller
        '''
        :param self: class referential variable
        :param error: distance from desired goal
        '''
        self.errors.append(error) #adds ost recent error to the list of errors
        if len(self.errors) > self.period: # removes the oldest error once the length of the list is greater than 40
            self.errors.pop(0)
        p = self.Kp * error#calculation for p value
        if len(self.errors) > 1:
            d = self.Kd * (error - self.errors[-2]) / self.dt# calculation for d
        else:
            d = 0 #d is 0 on first measurement
        i = self.Ki * sum(self.errors)/len(self.errors)*self.dt #calculation for i
        u = p + i + d #calculation for u

        self.pid_msg = Float64MultiArray() # does stuff
        self.pid_msg.data = [p, d, i] # does things 
        self.pidpub.publish(self.pid_msg) # does stuff and things whilst debugging

        return u #gives u to whatever asked for it
PID()# calls the program
