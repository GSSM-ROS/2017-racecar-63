#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class PID():
    def __init__(self, Kp, Kd, Ki, direction, goal):
        """ 
        Constructor for PID controller inits node stuff and spins
        """
        rospy.init_node("pid", anonymous=False)
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=17)
        rospy.Subscriber("/scan", LaserScan, self.callback)

        self.velocity = 2 #constant velocity
        self.Kp = Kp #proportional constant
        self.Kd = Kd #differential constant
        self.Ki = Ki #integral constant
        self.dt = 0.1 #time delta
        self.errors = [] #errors list
        self.period = 40 #period of sliding window on scan errors
        self.goal = goal #goal distance (m)

        self.right = True
        if direction == "left":
            self.right = False

        rospy.spin()

    def callback(self, msg):
        ''' 
        accquires a laser scan message, then uses the pid to publish a new steering angle
        also includes saftey control

        :param self: class-based reference object to have class-wide variables
        :param msg: a laser scan accquired from the topic
        '''
        self.send = AckermannDriveStamped()      #creates a message 

        self.slice = msg.ranges[50:540] if self.right else msg.ranges[540:1030] #sets slice for wall follow depending on variable in init
        error = self.goal - min(self.slice)	#creates error for pid

        u = self.pid(error)			#gets pid calculation
        self.send.drive.steering_angle = u if self.right else -u	#determines right or left steering

        self.send.drive.speed = self.velocity	#sets speed
	
        if min(msg.ranges[450:630]) < 0.5:	#safety for speed
            self.send.drive.steering_angle = -self.send.drive.steering_angle #negates steering to turn around corners
            self.send.drive.speed = -0.2		#brakes, reverses

        self.pub.publish(self.send)		#pubs message

    def pid(self, error):             #pid control - steering angle calculate
	'''
	Determines a steering variable dependant on the distance from the wall, previous distance changes, and distance consistancy over time
	(PID)
	
	:param self: class referential variable
	:param error: distance from desired goal
	'''
        self.errors.append(error)
        if len(self.errors) > self.period:  #keeps the # of averages < 40
            self.errors.pop(0)
        p = self.Kp * error     #proportional correction
        if len(self.errors) > 1:        #if it's < 1, set = 0
            d = self.Kd * (error - self.errors[-2]) / self.dt  #derivative
        else:
            d = 0
        i = self.Ki * sum(self.errors)/len(self.errors)*self.dt     #integral correction
        u = p + i + d   #error term
        return u

#Do main stuff now
Kp = rospy.get_param('/Kp')
Kd = rospy.get_param('Kd')
Ki = rospy.get_param('Ki')
direction = rospy.get_param('direction')
goal = rospy.get_param('goal')

PID(Kp, Kd, Ki, direction, goal) #init the node
