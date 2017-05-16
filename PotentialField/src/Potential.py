#!/usr/bin/python
import math #Because everyone needs some math
import rospy #Necessary for publishing
from sensor_msgs.msg import LaserScan #receives from the LIDAR
from ackermann_msgs.msg import AckermannDriveStamped #Controls steering


class Pot_Field(): #We are in RCS class. Everything we are doing is in this class. There are three gssm students in RCS and three methods in this class. A triangle has three sides. Coincidence? I think not. Illuminati confirmed.
    ''' 
    Potential Field class for node 'potential' to navigate in free space
    '''

    def drive(self, data): #Receives scans and controls steering and speed
        ''' 
        Drive callback recieves from LiDAR and drives with potential field

        Recieves from /scan and publishes to ackermann drive topic (of long name)
        :param data: LaserScan message recieved from /scan topic
        '''
        self.send=AckermannDriveStamped() #declares self.send as an ackermanndrivestamped. allows us to decide speed and turning angle
        self.slice=data.ranges[180:900] #Our slice contains the front 180 degrees of the LIDAR scan
        x=0 #x is an index position on self.slice
        pushback=0 #pushback is the force in the y(forward, backward) direction
        turn = 0 #turn is the force in the x(right, left) direction
        for x in range(len(self.slice)):#This is a for loop. It lets us do one thing many times.
            angle=x*.25#since there are 720 points in 180 degrees, each point is .25 degrees.
            pushback += math.sin(math.radians(angle))*self.point_charge/(self.slice[x])**2 #Equation for pushback. Uses kq/r**2 to determine amount the speed is reduced. Added together for each point.
            turn += math.cos(math.radians(angle))*self.point_charge/(self.slice[x])**2 #Equation for turn. Uses the same equation as above to determine turning angle. cumulative
        self.send.drive.speed=self.drive_constant * (self.back_force-pushback) #sets net force equal to backforce - pushback
        if self.send.drive.speed > self.max_speed:
            self.send.drive.speed = self.max_speed
        elif self.send.drive.speed < -self.max_speed:
            self.send.drive.speed = -self.max_speed
        self.send.drive.steering_angle = self.turn_constant * turn #sets net force = turn* a constant that scales the turning angle according to distance.
        if self.send.drive.speed < 0: #Checks if the speed is negative
            self.send.drive.steering_angle = -self.send.drive.steering_angle #reverses steering angle to turn around while in reverse
            #positive because left is positive but cos(angle) starts at positive on left
        self.pub.publish(self.send) #publish to drive train

    def __init__(self): #Defines the initializer
        ''' 
        Constructor for Pot_Field to create ros node 'potential' and initialize values for potential vield
        '''
        rospy.init_node("potential", anonymous=False)#Initiates the node.
        rospy.Subscriber("/scan", LaserScan, callback=self.drive) #Creates a subscriber to the /scan node
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=6) #creates a publisher to the location that controls speed and steering angle.
        rospy.on_shutdown(self.die) #calls back die() on death.

        self.back_force = 2.5 #constant force from the back that is the speed when there is no pushback assuming its value is less than the value for the max speed of the robot
        self.point_charge = 0.008 #sets a q value for objects. This is adjusted to prevent few points from stopping the robot completely.
        self.turn_constant = 1 #Turn constant can be modified to scale rate of turning up or down.
        self.drive_constant = 1 #drive constant can be modified to change effect of pushback. Scales up or down.
        self.max_speed = 1 #maximum speed the robot will go
        rospy.spin() #keeps the subscriber and publisher going until killed

    def die(self): #Death callback
        ''' 
        Function to call on node shutdown - only makes unfortunate remarks
        '''
        print("Potentialman Down!")#prints to confirm death

Pot_Field()        #Calls to run the class
        


