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
        # subscribe to RGB image
        self.imageSubscriber = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)
        # subscribe to Depth image                                  
        self.depthImageSub = rospy.Subscriber('/camera/depth/image_raw', Image,
                                          self.depth_image_callback)
        # publish movement commands                                  
        self.velocityPublisher = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)
        # placeholder                                    
        self.depth_image_CV2 = ""
        
        # assign new variable to self from Twist type
        self.twist = Twist()
        
        # placeholder values set for HSV masks
        self.maskRed = None
        self.maskGreen = None
        self.maskBlue = None
        self.maskYellow = None
        
        self.colourMasks = [self.maskRed, self.maskGreen, self.maskBlue,
                            self.maskYellow]
       
        # dictionary of visited colours
        self.coloursVisited = [False] * 4
               
        self.firstUnseenInd = 0
            
        # values from ROS image to an array    
    def depth_image_callback(self, data):
        # converts ROS depth image to numpy array
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data)
        

        
        #  this is the function starts every time when camera/rgb/image_raw is updated
    def image_callback(self, message):
        if numpy.sum(numpy.array(self.coloursVisited).astype('int')) == 4:
            self.twist.linear.x  = 0.0
            self.twist.angular.z = 0.0
            self.velocityPublisher.publish(self.twist)            
            cv2.destroyAllWindows()            
            exit()
        
        self.velocityPublisher.publish(self.twist)
        # print self.twist
        # create windows to display 
        cv2.namedWindow("RobotView", 1)
        cv2.namedWindow("SegmentedView", 2)
        # send message of an image to openCv as image 
        imageRobot = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        # convert BGR to HSV
        # convert as RGB color into Degrees and *2/3
        hsv = cv2.cvtColor(imageRobot, cv2.COLOR_BGR2HSV)
        hsvNoCircle = hsv
        # RED mask
        lowerRed = numpy.array([0, 70, 50])
        upperRed = numpy.array([6, 255, 255])
        self.maskRed = cv2.inRange(hsv, lowerRed, upperRed)
        # filter out image to avoid confusion
        self.maskRed = cv2.medianBlur(self.maskRed,7)
        # GREEN mask
        lowerGreen = numpy.array([60, 120, 100])
        upperGreen = numpy.array([70, 255, 255])
        self.maskGreen = cv2.inRange(hsv, lowerGreen, upperGreen)
        # filter out image to avoid confusion        
        self.maskGreen = cv2.medianBlur(self.maskGreen,7)
        # BLUE mask
        lowerBlue = numpy.array([110, 120, 100])
        upperBlue = numpy.array([130, 255, 255])
        # filter out image to avoid confusion        
        self.maskBlue = cv2.inRange(hsv, lowerBlue, upperBlue)
        # filter out image to avoid confusion
        self.maskBlue = cv2.medianBlur(self.maskBlue,7)
        # YELLOW mask
        lowerYellow = numpy.array([30, 120, 100])
        upperYellow = numpy.array([40, 255, 255])
        self.maskYellow = cv2.inRange(hsv, lowerYellow, upperYellow)
        # filter out image to avoid confusion        
        self.maskYellow = cv2.medianBlur(self.maskYellow,7)
        # correspond to the bool array
        self.colourMasks = [self.maskRed, self.maskGreen, self.maskBlue,
                            self.maskYellow]
                            
        #print(self.coloursVisited)
        
        # finds first unseen mask's index
        for i in range (0, len(self.coloursVisited)):
            if not self.coloursVisited[i]:
                self.firstUnseenInd = i
                break
        
        # unseenMasks defined as first unseen mask initially, then other unseen
        # masks are added
        unseenMasks = self.colourMasks[self.firstUnseenInd]
        for i in range (self.firstUnseenInd, len(self.coloursVisited)):
            if not self.coloursVisited[i]:            
                unseenMasks += self.colourMasks[i]
      
        # filter again with medain filter
        colourAvgFilt = cv2.blur(unseenMasks,(5,5)) 
        
        height, width, d = imageRobot.shape
        
        searchTop = height / 4
        searchBot = ((3*height)/4) + 20
        colourAvgFilt[0:searchTop, 0:width] = 0
        colourAvgFilt[searchBot:height, 0:width] = 0
        move = cv2.moments(colourAvgFilt)
        # dictionary for matching filter mask, 
        # like region probs like from matlab
        # contour area from the centroid for X and Y
        # so there are sums of X and the Sums of Y 
        if move['m00'] > 0:
            centerX = int(move['m10']/move['m00'])
            centerY = int(move['m01']/move['m00'])
            # cv2.circle(imageRobot, (centerX, centerY), 20, (255, 0, 255), -1)
            cv2.circle(imageRobot, (centerX, centerY), 20, (255, 0, 255), -1)
            error = centerX - width/2
            self.twist.linear.x = 1
            self.twist.angular.z = -float(error) / 100

            distanceToObject = self.depth_image_CV2[centerY, centerX]
            # print(distanceToObject)
            
            
            if distanceToObject < 1.0:
                self.twist.linear.x  = 0.0
                self.twist.angular.z = 0.5
                # print("Object found")
                # Error handling for if circle is not present
                try:
                    # finding HSV of centre of circle, and comparing with 
                    # thresholds to get colour found
                    centerHSV = hsvNoCircle[centerY, centerX]
                    # first if statement excludes errors in reading HSV values
                    print(centerHSV)
                    if not centerHSV[1] == 0 and not centerHSV[2] == 155:           
                        if (centerHSV[0] >= lowerRed[0] and centerHSV[0] <= upperRed[0]) or \
                        (centerHSV[0] >= 170 and centerHSV[0] <= 180):
                            self.coloursVisited[0] = True                            
                            print("Red found!")
                        elif centerHSV[0] >= lowerGreen[0] and centerHSV[0] <= upperGreen[0]:
                            self.coloursVisited[1] = True                            
                            print("Green found!")
                        elif centerHSV[0] >= lowerBlue[0] and centerHSV[0] <= upperBlue[0]:
                            self.coloursVisited[2] = True                            
                            print("Blue found!")
                        elif centerHSV[0] >= lowerYellow[0] and centerHSV[0] <= upperYellow[0]:                            
                            self.coloursVisited[3] = True                            
                            print("Yellow found!")
                except:
                    pass
        else:
            self.twist.angular.z = 0.5
            self.twist.linear.x  = 0.0        


            self.velocityPublisher.publish(self.twist)
        cv2.imshow("RobotView", imageRobot)
        cv2.imshow("SegmentedView", colourAvgFilt)
        cv2.imshow("MaskRed", self.maskRed)
        #cv2.imshow("MaskGreen", maskGreen)
        #cv2.imshow("MaskBlue", maskBlue)
        #cv2.imshow("MaskYellow", maskYellow)
        
        # how long it wait between refresh
        cv2.waitKey(1)


cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()


