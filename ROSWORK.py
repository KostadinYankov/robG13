#!/usr/bin/env python
# import 
import numpy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, PoseWithCovariance
from std_msgs.msg import Header
from time import sleep
#import datetime
#################################copy guy

class Follower:
    
    posRobot = PoseWithCovariance()
    PI = 3.1415926535897
    def __init__(self):
        self.moveSomewhere = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1) 
        #self.dataTaken = rospy.Subscriber('/amcl_pose', PoseWithCovariance, self.posCallBack )
        #print("first run")
        
        self.bridge = cv_bridge.CvBridge()
        # subscribe to RGB image
        self.imageSubscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # subscribe to Depth image                                  
        self.depthImageSub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        # publish movement commands                                  
        self.velocityPublisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)                                  
        # placeholder                                    
        self.depth_image_CV2 = ""
        
        # assign new variable to self from Twist type
        self.twist = Twist()
        # placeholder values set for HSV masks
        self.maskRed = None
        self.maskGreen = None
        self.maskBlue = None
        self.maskYellow = None
        
        self.colourMasks = [self.maskRed, 
                            self.maskGreen, 
                            self.maskBlue,
                            self.maskYellow]
       
        # dictionary of visited colours
        self.coloursVisited = [False] * 4
        self.counterPublish = 0
        self.firstUnseenInd = 0
            
        # values from ROS image to an array    
    def depth_image_callback(self, data):
        # converts ROS depth image to numpy array
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data)
        
        #  this is the function starts every time when camera/rgb/image_raw is updated
        
        #print("second run")
    #def spin(self):
        #self.rotate = True
        #if self.rotate:
            #self.startAngle = 
    #def posCallBack(self, data ):
        #self.posRobot = data
        # holds the position of robot
       # print(self.posRobot)
    def moveToGoal(self, x, y):
    
        if self.moveSomewhere.get_num_connections() == 1:
            if self.counterPublish == 0 :
                #PoseStamped 
                    # header
                    #   frame_id
                    # pose
                    #   position
                    #       x
                    #       y
                    #   orientation
                    #       x
                    #       y
                    #       z
                    #       w
                print("move to goal function")
                global positionPoint 
                positionPoint = Point()
                positionPoint.x = x
                positionPoint.y = y
                
                orientationQuaternion = Quaternion()
                orientationQuaternion.w = 1.0
                
                posePose = Pose()
                posePose.position = positionPoint
                posePose.orientation = orientationQuaternion
                
                headerHeader = Header()
                headerHeader.frame_id = 'map'
                headerHeader.stamp = rospy.Time.now()
                
                posePoseStamped = PoseStamped()
                posePoseStamped.header = headerHeader
                posePoseStamped.pose = posePose
                self.moveSomewhere.publish(posePoseStamped)
                
                print("3rd run")
                print('Published move goal to {0}, {1}.'.format(posePoseStamped.pose.position.x, posePoseStamped.pose.position.y))            
                
                self.counterPublish += 1
     
            # if this has happened then start look for a color
            print (self.counterPublish)
        
    def image_callback(self, message):
        global counterPublish
        global positionPoint
       
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
        self.colourMasks = [self.maskRed, 
                            self.maskGreen,
                            self.maskBlue,
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
# place object coordinates infron of the robot
# go towards objet coordinates ... this hapens automatically
# also stop before hitting object 
# it will navigate towards object and avoid obstacle
            
            distanceToObject = self.depth_image_CV2[centerY, centerX]
            print(distanceToObject)
            
            ####################################
            # print distance to object
            
            if distanceToObject < 1.0:
                print('Object is now: {0}m away.'.format(distanceToObject))
                
                self.twist.linear.x  = 0.0
                self.twist.angular.z = 0.5
                # print("Object found")
                # Error handling for if circle is not present
                try:
                    # finding HSV of centre of circle, and comparing with 
                    # thresholds to get colour found
                    centerHSV = hsvNoCircle[centerY, centerX]
                    # first if statement excludes errors in reading HSV values
                    #print(centerHSV)
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
                self.twist.linear.x = 1
        elif move['m00'] < 0:
            speed = 0.5
            print("twist because there is nothing else to do !!!!!")
            self.twist.angular.z = speed*self.PI/360
            self.twist.linear.x  = 0.0        
            self.velocityPublisher.publish(self.twist)        
            cv2.imshow("RobotView", imageRobot)
            cv2.imshow("SegmentedView", colourAvgFilt)       
            #cv2.imshow("MaskRed", self.maskRed)
            #cv2.imshow("MaskGreen", maskGreen)
            #cv2.imshow("MaskBlue", maskBlue)
            #cv2.imshow("MaskYellow", maskYellow)
            print("success with twist in 1 full spin")
            
        else:
            print("goal activated here")
            self.moveToGoal(-4.0, -4.0)
            #self.moveToGoal(5.0, 5.0)
            sleep(10)
        
            
        # how long it wait between refresh
            cv2.waitKey(1)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()


