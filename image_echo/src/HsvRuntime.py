#!/usr/bin/python
#imports
import rospy
from image_echo.msg import HSV

class HSVChanger():
    '''
    Allows a user to change HSV values on the fly
    '''
    def __init__(self):
        '''
        initializes the node
        :param self: the node itself
        '''
        rospy.init_node("HSVman", anonymous=False)              #creates the node
        self.pub = rospy.Publisher("/hsv", HSV, queue_size=1)   #makes a publisher
           
        rospy.on_shutdown(self.on_shutdown)                     #defines the shutdown function for killing the node 


        self.rate = 10                                          #rate of publishing
        r = rospy.Rate(self.rate)

        self.is_running = True
        while is_running:

            value = input("Input Upper H, S, or V (uh, us, uv) or Lower H, S, V (lh, ls, lv)")
            
            self.my_message = HSV()
            print(value)
            
            

    def on_shutdown(self):
        self.is_running = false
        print("ded")
