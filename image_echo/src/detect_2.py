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

#function to find a contour

class Detector():
    def __init__(self):
        #TODO must make self.xy_pub to publish to '/contour_xy' and self.img_pub to publish to '/contoured_img' (both with Image messages)
        rospy.init_node("follower", anonymous=False)
 
        rospy.Subscriber("/camera/left/image_rect_color",Image,self.left_contour_callback)
        rospy.Subscriber("/camera/right/image_rect_color",Image,self.right_contour_callback)
        self.left_xy_pub = rospy.Publisher("/left/contour_xy",Point,queue_size = 1)
        self.right_xy_pub = rospy.Publisher("/right/contour_xy",Point,queue_size = 1)

        self.left_image_pub = rospy.Publisher("/left_contour_image", Image, queue_size = 1)  #for running in realtime, message undetermined curren$
        self.right_image_pub = rospy.Publish("/right_contour_image", Image, queue_size = 1)

        rospy.on_shutdown(self.die)
        self.bridge = CvBridge()

        self.upper_hsv = (10,200,200)   #max values to find blob - should be 3 - h, s, v
        self.lower_hsv = (0,100,100)   #min values to find blob

        self.left_x = -1
        self.left_y = -1
        self.right_x = -1
        self.right_y = -1

        rospy.spin()

    def die(self):
        print("Follower program stopped")

    def depth_callback(self, msg):
        im = self.bridge.imgmsg_to_cv2(msg)
        #z = np.min(im[self.left_x - 10:self.left_x+10,  self.left_y-10: self.left_y+10])    #indexes the image.  left_x and left_y is the depth               
        #TODO use a range of values and find 2D minimum instead of using exact number
        dz = 30
        lower_x, upper_x, lower_y, upper_y = self.cast_indeces(self.left_x, self.left_y, dz, (900,900))#TODOim.shape)
        z_range = im[lower_x : upper_x, lower_y : upper_y].flatten()
        #print("=======================\n%s" % im)
        print((z_range)) #TODO remove
        non_nan_z = z_range[~np.isnan(z_range)]

        if len(non_nan_z) > 0:
            z = min(non_nan_z)
            # z = im[self.left_x, self.left_y]

            point = Point()   #an object with 3 variables
            point.z = z
            point.x = self.left_x
            point.y = self.left_y

            print("publishing to /contour_xyz with depth %s" % point.z) #TODO remove
            self.left_xyz_pub.publish(point)
        else:
            print("z is all nan") #TODO remove

    def cast_indeces(x, y, d, shape):
        '''
        Casts indeces for 2D array if necessary
        :param x: x position to be at center
        :param y: y position to be at center
        :param d: difference to add/subtract for range
        :param shape: shape of surrounding array 2-tuple
        :return: Returns lower_x, upper_x, lower_y, upper_y
        '''
        lower_x = self.lower(x, d, 0)
        upper_x = self.upper(x, d, shape[0])
        lower_y = self.lower(y, d, 0)
        upper_y = self.upper(y, d, shape[1])
        return lower_x, upper_x, lower_y, upper_y

    def upper(value, d, bound):
        new_val = value + d
        if new_val > bound: new_value = bound
        return new_val

    def lower(value, d, bound):
        new_val = value - d
        if new_val < bound: new_value = bound
        return new_val
        
    def find_contour(self, img, lhsv=(0,100,100), uhsv=(10,200,200), debug=True): #TODO debug to False
        if debug: print("Starging a contour detection") #TODO remove
        im = self.bridge.imgmsg_to_cv2(img)

        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)   #converts to hsv

        r1 = np.array(lhsv) #35,110,175             #numpy arrays for hsv detection
        r2 = np.array(uhsv)#95,255,255           #defaults are for tennis ball

        mask = cv2.inRange(hsv,r1,r2)               #applys bitmask for in range values

        mask = cv2.GaussianBlur(mask,(51,51),0)     #blurs image to smooth noise

        mask = cv2.erode(mask, (5,5), iterations=20) #erodes noise to focus on main contour

        Contours, h = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                                    #creates a list of contours
        if (len(Contours) ==0):
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
            print("Putting debug info")
            cv2.drawContours(im, [bigContour], -1, (255,0,255), 2)#draws the contours
            cv2.circle(im, (cX, cY), 12 ,(50, 50, 255), -1)    #draws the circle
            image_msg = self.bridge.cv2_to_imgmsg(im,"bgr8")
            self.image_pub.publish(image_msg)
            print("%f & %f" %(cX, cY))

        self.leftX, self.left_y = cX, cY 
        return

Detector()
