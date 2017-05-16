#!/usr/bin/python

#imports
import cv2
import imutils
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge, CvBridgeError
import rospy
#from image_echo.msg import Contour
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import threading
from ackermann_msgs.msg import AckermannDriveStamped
import time


#function to find a contour

class Detector():
    def __init__(self):
        rospy.init_node("follower", anonymous=False)
 
        rospy.Subscriber("/camera/left/image_rect_color",Image,self.threaded_find_contour)
        rospy.Subscriber("/camera/depth/image_rect_color",Image,self.depth_callback)
        self.left_xyz_pub = rospy.Publisher("/contour_xyz",Point,queue_size = 1)

        self.image_pub = rospy.Publisher("/contour_image", Image, queue_size = 1)  #for running in realtime, message undetermined curren$

        self.emergency_speed_pub = rospy.Publisher("/contour_speed", AckermannDriveStamped, queue_size=1) #for emergency stops

        rospy.on_shutdown(self.die)
        self.bridge = CvBridge()

        self.upper_hsv = (9,255,255)   #max values to find blob - should be 3 - h, s, v
        self.lower_hsv = (5,210,130)   #min values to find blob

        self.left_x = -1
        self.left_y = -1

        self.debug = True

        self.last_non_nan = 0
        self.emergency_threshold = 1

        rospy.spin()

    def die(self):
        print("Follower program stopped")

    def depth_callback(self, msg):
        im = self.bridge.imgmsg_to_cv2(msg) #convert to numpy array

        dz = 100 #amount to go in any direction from point for getting range (the slice is [x-dz : x+dz][y-dz:y+dz]

        lower_x, upper_x, lower_y, upper_y = self.cast_indeces(self.left_x, self.left_y, dz, im.shape)
        #creates upper and lower ranges by adding and subtracting dz to x and y then making sure everything is in range

        z_range = im[lower_x : upper_x,lower_x : upper_x].flatten()
        #gets the slice from uppper and lower values in x and y then flattens that to a linear array

        non_nan_z = z_range[np.isfinite(z_range)]
        #get boolean array of when z is not nan then mask z range to only include those values (~ is logical not)

        if len(non_nan_z) > 0:
            #make sure there is something after the removal of nan

            z = min(non_nan_z)
            # z = im[self.left_x, self.left_y]

            point = Point()   #an object with 3 variables
            point.z = z #depth dimension of point
            point.x = self.left_x #x location from left camera
            point.y = self.left_y #y location from left camera

            self.left_xyz_pub.publish(point) #publish the point
            self.last_non_nan = time.time() #use now time as most recent good thing
        else:
            print("z not Israel") #Israeli ambassador has left the building
            self.emergency() #it might be an emergency

    def emergency(self):
        '''
        In case of emergency, publishes to speed message
        '''
        if time.time() > self.last_non_nan + self.emergency_threshold:
            # time has run out
            emergency_msg = AckermannDriveStamped()
            emergency_msg.drive.speed = 0 #zero speed
            emergency_msg.drive.steering_angle = 0 #zero angle
            self.emergency_speed_pub.publish(emergency_msg) #publish stop signal

    def cast_indeces(self, x, y, d, shape):
        '''
        Creates a range around a single point and casts indeces for 2D array if necessary
        :param x: x position to be at center
        :param y: y position to be at center
        :param d: difference to add/subtract for range
        :param shape: shape of surrounding array 2-tuple
        :return: Returns lower_x, upper_x, lower_y, upper_y
        '''
        lower_x = self.lower(x, d, 0) #get lower x point or zero
        upper_x = self.upper(x, d, shape[1]) #get upper x or width dimension
        lower_y = self.lower(y, d, 0) #get lower y point or zero
        upper_y = self.upper(y, d, shape[0]) #get upper y or height dimension
        return lower_x, upper_x, lower_y, upper_y

    def upper(self, value, d, bound):
        '''
        Gets value + d or the bound if that is out of range
        :param value: number to expand from
        :param d: change to add to the value
        :param bound: upper bound on amount
        :return: returns the new value
        '''
        new_val = value + d
        if new_val > bound: 
            new_val = bound
        return new_val

    def lower(self, value, d, bound):
        '''
        Get value - d or bound if that is out of range
        :param value: number to subtract from
        :param d: amount to subtract
        :param bound: lower bound on amount
        :return: returns the new value
        '''
        new_val = value - d
        if new_val < bound: 
            new_val = bound
        return new_val
        
    def threaded_find_contour(self, img):
        '''
        Callback to find countours and publish but using threads to reduce time
        :param img: Image message from topic to analyze
        '''
        thread = threading.Thread(target=self.find_contour, args=(img, self.lower_hsv, self.upper_hsv , self.debug))
        thread.setDaemon(True)
        thread.start()

    def find_contour(self, img, lhsv=(5,210,130), uhsv=(9,255,255), debug=False):
        '''
        Finds largest contour in an image using cv2 and saves them to object variables
        :param img: Image object
        :param lhsv: tuple with lower hsv range
        :param uhsv: tuple with upper hsv range
        :param debug: if true, it will print debug stuff and publish the overlaid image
        '''
        im = self.bridge.imgmsg_to_cv2(img)

        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)   #converts to hsv

        r1 = np.array(lhsv) #35,110,175             #numpy arrays for hsv detection
        r2 = np.array(uhsv)#95,255,255           #defaults are for tennis ball

        mask = cv2.inRange(hsv,r1,r2)               #applys bitmask for in range values

        mask = cv2.GaussianBlur(mask,(21,21),0)     #blurs image to smooth noise

        mask = cv2.erode(mask, (5,5), iterations=5) #erodes noise to focus on main contour

        Contours, h = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                                    #creates a list of contours
        if (len(Contours) ==0):
            print("No contours detected")
            self.emergency()
            return

        bigContour = Contours[0]                    #says biggest contour is first
        bigArea = cv2.contourArea(Contours[0])


        for i in Contours:                          #simple for loop algorith to check for largest contour area
            checkArea = cv2.contourArea(i)
            if checkArea > bigArea:
                bigArea = checkArea
                bigContour = i

        M = cv2.moments(bigContour)                 #creates moments of contour
        cX = int(M["m10"]/M["m00"])                 #uses moments for the circle
        cY = int(M["m01"]/M["m00"])

        xy_msg = Point() # = Contour()
        xy_msg.y = cY #contour y
        xy_msg.x = cX #contour x
        #xy_msg.height, xy_msg.width = mask.shape #height and width of image mask
        #publisher.publish(xy_msg)

        if debug:
            cv2.drawContours(im, [bigContour], -1, (255,0,255), 2)#draws the contours
            cv2.circle(im, (cX, cY), 12 ,(50, 50, 255), -1)    #draws the circle
            image_msg = self.bridge.cv2_to_imgmsg(im,"bgr8")
            self.image_pub.publish(image_msg)

        self.left_x, self.left_y = cX, cY 

Detector()
