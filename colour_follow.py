#!/usr/bin/env python
# import 
import numpy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        self.depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image,
                                          self.depth_image_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        self.depth_image_CV2 = ""
            #print(depth_image_CV2)
        
        self.twist = Twist()
        tw = Twist()
        tw.angular.z = twist

    def depth_image_callback(self, data):
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data)


    def image_callback(self, message):
        # create windows to display 
        cv2.namedWindow("RobotView", 1)
        cv2.namedWindow("SegmentedView", 2)
        # send message of an image to openCv as image 
        imageRobot = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        # convert BGR to HSV
        # convert as RGB color into Degrees and *2/3
        hsv = cv2.cvtColor(imageRobot, cv2.COLOR_BGR2HSV)
        # RED mask
        lowerRed = numpy.array([0, 120, 100])
        upperRed = numpy.array([6, 255, 255])
        maskRed = cv2.inRange(hsv, lowerRed, upperRed)
        maskRed = cv2.medianBlur(maskRed,7)
        # GREEN mask
        lowerGreen = numpy.array([110, 120, 100])
        upperGreen = numpy.array([130, 255, 255])
        maskGreen = cv2.inRange(hsv, lowerGreen, upperGreen)
        maskGreen = cv2.medianBlur(maskGreen,7)
        # BLUE mask
        lowerBlue = numpy.array([60, 120, 100])
        upperBlue = numpy.array([70, 255, 255])
        maskBlue = cv2.inRange(hsv, lowerBlue, upperBlue)
        maskBlue = cv2.medianBlur(maskBlue,7)
        # YELLOW mask
        lowerYellow = numpy.array([30, 120, 100])
        upperYellow = numpy.array([40, 255, 255])
        maskYellow = cv2.inRange(hsv, lowerYellow, upperYellow)
        maskYellow = cv2.medianBlur(maskYellow,7)
        
        allMasks= maskRed+maskGreen+maskBlue+maskYellow
        # use mean filter for noise such as salt and pepper (issue faced, and sorted)
         
        # filter again with medain filter
        colourAvgFilt = cv2.blur(allMasks,(5,5)) 
        
        height, width, d = imageRobot.shape
        
        searchTop = height / 4
        searchBot = ((3*height)/4) + 20
        colourAvgFilt[0:searchTop, 0:width] = 0
        colourAvgFilt[searchBot:height, 0:width] = 0
        move = cv2.moments(colourAvgFilt)
        if move['m00'] > 0:
            cx = int(move['m10']/move['m00'])
            cy = int(move['m01']/move['m00'])
            # cv2.circle(imageRobot, (cx, cy), 20, (255, 0, 255), -1)
            cv2.circle(imageRobot, (cx, cy), 20, (255, 0, 255), -1)
            error = cx - width/2
            self.twist.linear.x = 1
            self.twist.angular.z = -float(error) / 100

            distanceToObject = self.depth_image_CV2[cy, cx]
            print(distanceToObject)

            if distanceToObject < 1.0:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0


            #self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("RobotView", imageRobot)
        cv2.imshow("SegmentedView", colourAvgFilt)
        cv2.imshow("MaskRed", maskRed)
        cv2.imshow("MaskGreen", maskGreen)
        cv2.imshow("MaskBlue", maskBlue)
        cv2.imshow("MaskYellow", maskYellow)
        cv2.waitKey(3)


cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()


