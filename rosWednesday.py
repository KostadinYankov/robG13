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
from move_base_msgs.msg import MoveBaseActionGoal
from random import random

#import datetime
#################################copy guy

class Follower:
    
    #posRobot = PoseWithCovariance()
    #PI = 3.1415926535897
    def __init__(self):
        self.moveSomewhere = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=2)

        self.countSpin = 0
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


        self.listOfPoints = [(4,-5),(1, 0),(3.5,2.5), (4, 3.5)]
        self.listCount = 0
        self.activateSearch = True
        sleep(2)

        self.main()




    # values from ROS image to an array
    def depth_image_callback(self, data):
        # converts ROS depth image to numpy array
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data)



    # this is the function starts every time when camera/rgb/image_raw is updated
    def image_callback(self, message):
        self.image_data = message




    def moveToGoal(self, x, y):
        goSomewhere = MoveBaseActionGoal()
        #self.activateSearch = False
        goSomewhere.goal.target_pose.header.seq = 0
        goSomewhere.goal.target_pose.header.stamp = rospy.Time.now()
        goSomewhere.goal.target_pose.header.frame_id = "map"

        goSomewhere.goal.target_pose.pose.position.x = x
        goSomewhere.goal.target_pose.pose.position.y = y
        goSomewhere.goal.target_pose.pose.orientation.w = 1

        self.moveSomewhere.publish(goSomewhere)

        sleep(25)
        print("25 Seconds Reached")


    def generateMasks(self):
        self.velocityPublisher.publish(self.twist)
        # print self.twist
        # create windows to display
        cv2.namedWindow("RobotView", 1)
        cv2.namedWindow("SegmentedView", 2)
        # send message of an image to openCv as image
        self.imageRobot = self.bridge.imgmsg_to_cv2(self.image_data, desired_encoding='bgr8')
        # convert BGR to HSV
        # convert as RGB color into Degrees and *2/3
        hsv = cv2.cvtColor(self.imageRobot, cv2.COLOR_BGR2HSV)
        self.hsvNoCircle = hsv
        # RED mask
        self.lowerRed = numpy.array([0, 70, 50])
        self.upperRed = numpy.array([6, 255, 255])
        self.maskRed = cv2.inRange(hsv, self.lowerRed, self.upperRed)
        # filter out image to avoid confusion
        self.maskRed = cv2.medianBlur(self.maskRed, 7)
        # GREEN mask
        self.lowerGreen = numpy.array([60, 120, 100])
        self.upperGreen = numpy.array([70, 255, 255])
        self.maskGreen = cv2.inRange(hsv, self.lowerGreen, self.upperGreen)
        # filter out image to avoid confusion
        self.maskGreen = cv2.medianBlur(self.maskGreen, 7)
        # BLUE mask
        self.lowerBlue = numpy.array([110, 120, 100])
        self.upperBlue = numpy.array([130, 255, 255])
        # filter out image to avoid confusion
        self.maskBlue = cv2.inRange(hsv, self.lowerBlue, self.upperBlue)
        # filter out image to avoid confusion
        self.maskBlue = cv2.medianBlur(self.maskBlue, 7)
        # YELLOW mask
        self.lowerYellow = numpy.array([30, 120, 100])
        self.upperYellow = numpy.array([40, 255, 255])
        self.maskYellow = cv2.inRange(hsv, self.lowerYellow, self.upperYellow)
        # filter out image to avoid confusion
        self.maskYellow = cv2.medianBlur(self.maskYellow, 7)
        # correspond to the bool array
        self.colourMasks = [self.maskRed,
                            self.maskGreen,
                            self.maskBlue,
                            self.maskYellow]
        return self.colourMasks




    def main(self):
        while not rospy.is_shutdown():

            #if not looking for objects
            if not self.activateSearch:
            #end program if all 4 colours are found
                if numpy.sum(numpy.array(self.coloursVisited).astype('int')) == 4:
                    self.twist.linear.x  = 0.0
                    self.twist.angular.z = 0.0
                    self.velocityPublisher.publish(self.twist)
                    cv2.destroyAllWindows()
                    exit()


            #generate the masks
                self.colourMasks = self.generateMasks()

            #saves the found colour
                for i in range (0, len(self.coloursVisited)):
                    if not self.coloursVisited[i]:
                        self.firstUnseenInd = i
                        break

            #unseenMasks defined as first unseen mask initially, then other unseen
            #masks are added to img and cleaned
                unseenMasks = self.colourMasks[self.firstUnseenInd]
                for i in range (self.firstUnseenInd, len(self.coloursVisited)):
                    if not self.coloursVisited[i]:
                        unseenMasks += self.colourMasks[i]
                colourAvgFilt = cv2.blur(unseenMasks,(5,5))

                height, width, d = self.imageRobot.shape

                searchTop = height / 4
                searchBot = ((3*height)/4) + 20
                colourAvgFilt[0:searchTop, 0:width] = 0
                colourAvgFilt[searchBot:height, 0:width] = 0
                move = cv2.moments(colourAvgFilt)
                # dictionary for matching filter mask,
                # like region probs like from matlab
                # contour area from the centroid for X and Y
                # so there are sums of X and the Sums of Y
                #/mobile_base/events/bumper
            # use that here to see if bump into something
                # if there are any objects in the mask
                if move['m00'] > 0:
                    print(move['m00'])
                    print("finding object")
                    # move towards the object
                    centerX = int(move['m10']/move['m00'])
                    centerY = int(move['m01']/move['m00'])
                    cv2.circle(self.imageRobot, (centerX, centerY), 20, (255, 0, 255), -1)
                    distanceToObject = self.depth_image_CV2[centerY, centerX]


                    if distanceToObject > 1.0 or numpy.isnan(distanceToObject):
                        error = centerX - width/2

                        self.twist.linear.x = 1
                        self.twist.angular.z = -float(error) / 100

                        self.velocityPublisher.publish(self.twist)
                        print("go speed 1 ")
                        #sleep(1)


                    #if the object in view is less then 1m away
                    elif distanceToObject <= 1.0:
                        print('Object is now: {0}m away.'.format(distanceToObject))
                        #self.twist.linear.x = 0
                        #sleep(1)


                            #print(self.end-self.start)
                        # print("Object found")
                        # Error handling for if circle is not present
                        try:
                            # finding HSV of centre of circle, and comparing with
                            # thresholds to get colour found
                            centerHSV = self.hsvNoCircle[centerY, centerX]
                            # first if statement excludes errors in reading HSV values
                            #print(centerHSV)
                            if not centerHSV[1] == 0 and not centerHSV[2] == 155:
                                if (centerHSV[0] >= self.lowerRed[0] and centerHSV[0] <= self.upperRed[0]) or \
                                (centerHSV[0] >= 170 and centerHSV[0] <= 180):
                                    self.coloursVisited[0] = True
                                    print("Red found!")
                                elif centerHSV[0] >= self.lowerGreen[0] and centerHSV[0] <= self.upperGreen[0]:
                                    self.coloursVisited[1] = True
                                    print("Green found!")
                                elif centerHSV[0] >= self.lowerBlue[0] and centerHSV[0] <= self.upperBlue[0]:
                                    self.coloursVisited[2] = True
                                    print("Blue found!")
                                elif centerHSV[0] >= self.lowerYellow[0] and centerHSV[0] <= self.upperYellow[0]:
                                    self.coloursVisited[3] = True
                                    print("Yellow found!")

                        except:
                            pass
                else:
                    print("spin here with z 1")
                    t = Twist()
                    self.twist.angular.z = 1
                    self.velocityPublisher.publish(t)
                    self.countSpin += 1
                    #sleep(1)
                    print(self.countSpin)
                    if self.countSpin == 200:
                        break
                    else:
                        move['m00'] > 0

            else:
                # if not the last element of the list, move again
                if self.listCount <= len(self.listOfPoints):
                    # fist element of the list of points self.activate = False
                    self.activateSearch = False
                    self.moveToGoal(self.listOfPoints[self.listCount][0], self.listOfPoints[self.listCount][1])
                    self.listCount += 1
                else:
                    # if it is last element quit program
                    print("finish program")
                    self.twist.linear.x  = 0.0
                    self.twist.angular.z = 0.0
                    self.velocityPublisher.publish(self.twist)
                    cv2.destroyAllWindows()
                    exit()

            #move['m00'] < 0:
             #   speed = 0.5
              #  print("twist because there is nothing else to do !!!!!")
               # self.twist.angular.z = speed*self.PI/360
                #self.twist.linear.x  = 0.0
                #self.velocityPublisher.publish(self.twist
                    cv2.imshow("RobotView", imageRobot)
                    cv2.imshow("SegmentedView", colourAvgFilt)





                    # how long it wait between refresh
                    cv2.waitKey(1)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()

