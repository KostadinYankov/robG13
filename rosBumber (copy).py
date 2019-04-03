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
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal, MoveBaseAction
from random import random
from tf.transformations import quaternion_from_euler
import actionlib
from actionlib_msgs.msg import *
from kobuki_msgs.msg import BumperEvent
class Follower: # create class
    

    def __init__(self): # initialisisation
        # publisher moveSomewhere with topic to publish moveBaseGoal, type of message and queue size
        self.moveSomewhere = rospy.Publisher('/move_base/goal', MoveBaseActionGoal, queue_size=2)
        # counts the spins when the robot turns
        self.countSpin = 0
        #
        self.moveBase = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # ros library to convert ROS images to OpenCV
        self.bridge = cv_bridge.CvBridge()
        # subscribe to RGB image
        self.imageSubscriber = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # subscribe to Depth image                                  
        self.depthImageSub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)
        # publish movement commands                                  
        self.velocityPublisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)                                  
        # placeholder for later on to hold the depth of the point                                   
        self.depth_image_CV2 = ""
        # assign new variable to self from Twist type
        self.twist = Twist()
        self.msg = Twist()
        # placeholder values set for HSV masks
        self.maskRed = None
        self.maskGreen = None
        self.maskBlue = None
        self.maskYellow = None
        # array of the masks
        self.colourMasks = [self.maskRed, 
                            self.maskGreen, 
                            self.maskBlue,
                            self.maskYellow]
        # dictionary of visited colours
        self.coloursVisited = [False] * 4
        #self.counterPublish = 0
        # unseen colour variable for later on to be stored in to
        self.firstUnseenInd = 0
        # array of the desired points to visit
        self.listOfPoints = [(4,-4),(-4,-4),(0.5, -0.2), (-1.5, 3.5), (-4,1.6),(-4.5,5),(-4,1),(-4.5,4),]
        # counter for visited points 
        self.listCount = 0
        # boolian variable to check if the searching function is on 
        self.searchObjects = True
        #sleep(2)
        self.main() # initialise main function
        #################################################
        #self.onBump= None
        # bump susbscriber
        #self.bumpSub = rospy.Subscriber(('/mobile_base/events/bumper', BumperEvent, self.bumpFunction)

        #self.velocityPublisher.publish(msg)

        
    """def bumpFunction(self,msg):
        if msg.state != BumperEvent.PRESSED:
            return
        if msg.bumper not in [BumperEvent.CENTER, BumperEvent.LEFT, BumperEvent.RIGHT]:
            return
        if self.onBump is not None:
            self.onBump.__call__()"""


    # values from ROS image to an array of data
    def depth_image_callback(self, data):
        # converts ROS depth image to numpy array
        self.depth_image_CV2 = self.bridge.imgmsg_to_cv2(data)

    # this is the function starts every time when camera/rgb/image_raw is updated
    def image_callback(self, message):
        self.image_data = message

    # moving to a desired place in the map function
    def moveToGoal(self, x, y):
        print("Moving to next position")
        self.searchObjects = False
        goSomewhere = MoveBaseActionGoal()
        goSomewhere.goal.target_pose.header.seq = 0
        goSomewhere.goal.target_pose.header.stamp = rospy.Time.now()
        goSomewhere.goal.target_pose.header.frame_id = "map"
        goSomewhere.goal.target_pose.pose.position.x = x
        goSomewhere.goal.target_pose.pose.position.y = y
        goSomewhere.goal.target_pose.pose.orientation.w = 1
        self.moveSomewhere.publish(goSomewhere)

        sleep(30)
        #print("30 Seconds Reached")
        print("move goal finished")
        #self.searchObjects =True

    def goForwardAndAvoid(self,x,z):
        print("Go forward and AVOID started :")
        self.searchObjects = True
        # converts the angle in radians to 4 dimentianal input for Twist
        # z is calculated as an angle .... radians ???
        angle = quaternion_from_euler(0,0,z*(numpy.pi/180))

        # send goal to move forward
        goal = MoveBaseActionGoal()
        goal.goal.target_pose.header.seq = 0
        goal.goal.target_pose.header.frame_id = 'base_link'
        goal.goal.target_pose.header.stamp = rospy.Time.now()
        goal.goal.target_pose.pose.position.x = x # meters we want the robot to go forward
        goal.goal.target_pose.pose.orientation.w = angle[3]
        goal.goal.target_pose.pose.orientation.z = angle[2]

        self.moveSomewhere.publish(goal)
        sleep(10)
        print("Go forward and AVOID finished")







    def generateMasks(self):
        self.velocityPublisher.publish(self.twist)
        #self.moveSomewhere.publish(self.goal)
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
            # end program if all 4 colours are found
            if numpy.sum(numpy.array(self.coloursVisited).astype('int')) == 4:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.velocityPublisher.publish(self.twist)
                cv2.destroyAllWindows()
                exit()
            # generate the masks
            self.colourMasks = self.generateMasks()
            # saves the found colour
            for i in range(0, len(self.coloursVisited)):
                if not self.coloursVisited[i]:
                    self.firstUnseenInd = i
                    break

            # unseenMasks defined as first unseen mask initially, then other unseen
            # masks are added to img and cleaned
            unseenMasks = self.colourMasks[self.firstUnseenInd]
            for i in range(self.firstUnseenInd, len(self.coloursVisited)):
                if not self.coloursVisited[i]:
                    unseenMasks += self.colourMasks[i]
            colourAvgFilt = cv2.blur(unseenMasks, (5, 5))

            height, width, d = self.imageRobot.shape

            searchTop = height / 4
            searchBot = ((3 * height) / 4) + 20
            colourAvgFilt[0:searchTop, 0:width] = 0
            colourAvgFilt[searchBot:height, 0:width] = 0
            move = cv2.moments(colourAvgFilt)
            # dictionary for matching filter mask,
            # like region probs like from matlab
            # contour area from the centroid for X and Y
            # so there are sums of X and the Sums of Y
            # /mobile_base/events/bumper

            #if not looking for objects
            if not self.searchObjects:
                # use that here to see if bump into something
                # if there are any objects in the mask
                if move['m00'] > 0:
                    #print(move['m00'])
                    #print("finding object")
                    # move towards the object
                    centerX = int(move['m10']/move['m00'])
                    centerY = int(move['m01']/move['m00'])
                    cv2.circle(self.imageRobot, (centerX, centerY), 20, (255, 0, 255), -1)
                    distanceToObject = self.depth_image_CV2[centerY, centerX]

                    # if the object in view is more than 1m away
                    if distanceToObject > 1.0 or numpy.isnan(distanceToObject):
                        error = centerX - width/2
                        print(distanceToObject)

                        self.twist.angular.z = -float(error) / 100
                        self.velocityPublisher.publish(self.twist)

                        if (unseenMasks[240,320] != 0):
                            print("object centered")
                            self.goForwardAndAvoid(distanceToObject-0.5, 0)
                            sleep(10)


                    #if the object in view is less than 1m away
                    elif distanceToObject <= 1.0:
                        print('Object is now: {0}m away.'.format(distanceToObject))
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        # asuring that only 1 object will be detected at a time
                        self.countSpin= 200
                        #sleep(1)



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
                    if self.countSpin < 200:
                        #self.countSpin = 0
                        #print("spin here with z 1")
                        t = Twist()
                        self.twist.angular.z = 1
                        self.velocityPublisher.publish(t)
                        self.countSpin += 1
                        #sleep(1)
                        #print(self.countSpin)

                        #self.coloursVisited =+ 1
                    else:
                        if numpy.any(self.coloursVisited == True) and distanceToObject <=1 and self.countSpin <=200:
                            t = Twist()
                            #self.twist.angular.z = 1
                            self.velocityPublisher.publish(t)
                            #self.countSpin +=
                            self.twist.linear.x = 0
                            self.twist.angular.z = 0
                            #self.countSpin = 200
                            
                            #self.twist.angular.z = 0
                        elif self.listCount <= len(self.listOfPoints):
                            # fist element of the list of points self.activate = False
                            self.moveToGoal(self.listOfPoints[self.listCount][0], self.listOfPoints[self.listCount][1])
                            self.listCount += 1
                            self.countSpin = 0
                            
                        else:
                            # if it is last element quit program
                            print("finish program")
                            self.twist.linear.x = 0.0
                            self.twist.angular.z = 0.0
                            self.velocityPublisher.publish(self.twist)
                            cv2.destroyAllWindows()
                            exit()

            else:
                # if not the last element of the list, move again
                if self.listCount <= len(self.listOfPoints):
                    # fist element of the list of points self.activate = False
                    #self.searchObjects = False
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

                    #cv2.imshow("RobotView", imageRobot)
                    cv2.imshow("SegmentedView", colourAvgFilt)





                    # how long it wait between refresh
                    cv2.waitKey(1)

cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()

cv2.destroyAllWindows()

