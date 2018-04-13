#! /usr/bin/env python
from __future__ import print_function
from __future__ import division
import rospy
#from openpose_msgs.msg import Estimations
from openpose_ros_wrapper_msgs.msg import Persons
from openpose_ros_wrapper_msgs.msg import PersonDetection
from openpose_ros_wrapper_msgs.msg import BodyPartDetection
import re
import math
import actionlib
import activity_detection.msg


NOSE = 0
NECK = 1
R_SHOULDER =2
R_ELBOW = 3
R_WRIST = 4
L_SHOULDER= 5
L_ELBOW = 6
L_WRIST = 7
R_HIP = 8
R_KNEE = 9
R_ANKLE = 10
L_HIP = 11
L_KNEE = 12
L_ANKLE = 13
R_EYE = 14
L_EYE = 15
R_EAR = 16
L_EAR = 17
NODES = 18

ACUTE1 = 45
ACUTE2 = 80
OBTUSE1 = 100
OBTUSE2 = 135
LAYING_THRESHOLD = 1.2
STANDING_THRESHOLD = 0.45
Y_LEFT_POINT_THRESHOLD = 0.9
X_LEFT_POINT_THRESHOLD = 0.65
Y_LEFT_RAISE_THRESHOLD = 0.9
Y_RIGHT_POINT_THRESHOLD = 0.9
X_RIGHT_POINT_THRESHOLD = 0.65
Y_RIGHT_RAISE_THRESHOLD = 0.9

class ActivityDetection_Action:
#Node index
    global NOSE
    global NECK
    global R_SHOULDER
    global R_ELBOW
    global R_WRIST
    global L_SHOULDER
    global L_ELBOW
    global L_WRIST
    global R_HIP
    global R_KNEE
    global R_ANKLE
    global L_HIP
    global L_KNEE
    global L_ANKLE
    global R_EYE
    global L_EYE
    global R_EAR
    global L_EAR
    global NODES

    global ACUTE1
    global ACUTE2
    global OBTUSE1
    global OBTUSE2
    global LAYING_THRESHOLD
    global STANDING_THRESHOLD
    global Y_LEFT_POINT_THRESHOLD
    global X_LEFT_POINT_THRESHOLD
    global Y_LEFT_RAISE_THRESHOLD
    global Y_RIGHT_POINT_LOWER_THRESHOLD
    global Y_RIGHT_POINT_UPPER_THRESHOLD
    global X_RIGHT_POINT_LOWER_THRESHOLD
    global X_RIGHT_POINT_UPPER_THRESHOLD
    global Y_RIGHT_RAISE_THRESHOLD

    def __init__(self, name):
        #self.detector = rospy.ServiceProxy("extract_objects", BoundingBoxesForClasses)
        #self.detector.wait_for_service(10)
        self._as = actionlib.SimpleActionServer(name, activity_detection.msg.activityDetectionAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.last_image = None

        rospy.Subscriber('/openpose/pose',Persons,self.estimation_callback)
        #self.estimation_msg=Estimations()
        self.people_size = 0


    def maxDifference(self, array):
        global NODES
        diff = [0.0 for ix in range(NODES)]
        maxD = -500.0
        for i in range(NODES):                                                          #for all nodes
            for j in range(NODES):                                                      #for all nodes
                if((array[i]!= 0.0) and (array[j]!=0.0)):
                    diff[j] = abs(array[i] - array[j])
                    if(diff[j] >= maxD):
                        maxD = diff[j]
        return maxD

    def highestY(self,array):
        global NODES
        minY = 10000
        minYindex = NECK
        for i in range(NODES):
            if(array[i]<minY and array[i]>0):
                minY = array[i]
                minYindex = i
        return minYindex

    def personPose(self, pose, leftLeg, rightLeg):
        global LAYING_THRESHOLD, STANDING_THRESHOLD
        
        posture = "Can't identify posture"
        if(pose>LAYING_THRESHOLD):                                                      #used to determine if laying or not
            posture = "Laying down"                                                    #sets activity to standing
        elif(leftLeg != 0 or rightLeg !=0):                                             #checks to see if both legs are visible, may not work for amputated people
            if(leftLeg >= STANDING_THRESHOLD or rightLeg >= STANDING_THRESHOLD):        #thresholds for one leg to be extended upright
                posture =  "Standing"                                                  #sets activity to standing
            elif(leftLeg <STANDING_THRESHOLD or rightLeg <STANDING_THRESHOLD):          #if not standing or laying
                posture =  "Sitting down"                                              #sets activity to sitting
        return posture

    def leftArm(self, x, y, angle, isLaying):
        global Y_LEFT_POINT_THRESHOLD,X_LEFT_POINT_THRESHOLD, Y_LEFT_RAISE_THRESHOLD, ACUTE1, ACUTE2, OBTUSE1, OBTUSE2
        gesture = ""
        if(y < Y_LEFT_POINT_THRESHOLD):                                                 #threshold for height for pointing left
            if(x>X_LEFT_POINT_THRESHOLD):                                               #threshold for width for pointing left
                gesture = ", Pointing left"                                              #set activity to pointing left
        if(y >Y_LEFT_RAISE_THRESHOLD and isLaying==0):                                  #threshold for height for raising left hand, can't be done while laying
            gesture = gesture + ", Raising left hand"                                              #set activity to raising left hand
        #if((angle > ACUTE1 and angle<ACUTE2) or (angle >OBTUSE1 and angle<OBTUSE2)):    #threshold for waving left arm
            gesture = gesture +  ", Waving left arm"                                               #set activity for waving left arm
        return gesture

    def rightArm(self, x, y, angle, isLaying):
        global Y_RIGHT_POINT_LOWER_THRESHOLD, Y_RIGHT_POINT_UPPER_THRESHOLD, X_RIGHT_POINT_LOWER_THRESHOLD, X_RIGHT_POINT_UPPER_THRESHOLD, Y_RIGHT_RAISE_THRESHOLD, ACUTE1, ACUTE2, OBTUSE1, OBTUSE2
        gesture = ""
        if(y <Y_RIGHT_POINT_THRESHOLD):      #threshold for height for pointing right
            if(x>X_RIGHT_POINT_THRESHOLD):   #threshold for width for pointing right
                gesture = ", Pointing right"                                             #set activity to pointing right
        if(y >Y_RIGHT_RAISE_THRESHOLD and isLaying==0):                                 #threshold for height for raising right hand, can't be done while laying
            gesture =gesture + ", Raising right hand"                                              #set activity to raising right hand
        if((angle > ACUTE1 and angle<ACUTE2) or (angle >OBTUSE1 and angle<OBTUSE2)):    #threshold for waving right arm
            gesture = gesture +  ", Waving right arm"                                              #set activity to waving right arm
        return gesture

    def nodeDifference(self, array, num1, num2):
        return abs(array[num1] - array[num2])

    def process_person(self, person):
        global NODES, R_WRIST, NECK, L_WRIST, R_KNEE, L_KNEE, L_HIP, L_ANKLE, R_HIP, R_ANKLE, R_ELBOW, L_ELBOW
        activity = "TEST"                                                         #string to state activities done by person
        isLaying =0;
        xCoord = [0.0 for i in range(NODES)]
        yCoord = [0.0 for i in range(NODES)]

        for i in range(NODES):
            xCoord[i] = person.body_part[i].x
            yCoord[i] = person.body_part[i].y
                
        maxY = self.maxDifference(yCoord)                                                    #maximum difference for x coordinates
        maxX = self.maxDifference(xCoord)                                                    #maximum difference for y coordinates
        topY = self.highestY(yCoord)

        xL = 0                                                                         #dummy initialization
        xR = 0                                                                         #dummy initialization
        yL =0                                                                          #dummy initialization
        yR = 0                                                                         #dummy initialization
        yLeftLeg = 0.0                                                                   #dummy initialization
        yRightLeg = 0.0                                                                  #dummy initializatio
        pose = 0.0                                                                       #dummy initialization

        #xRHeadHip = self.nodeDifference(xCoord, R_HIP, NECK)
        #xLHeadHip = self.nodeDifference(xCoord, L_HIP, NECK)
        yRHeadHip = self.nodeDifference(yCoord, R_HIP, topY)
        yLHeadHip = self.nodeDifference(yCoord, L_HIP, topY)

        xRHandHead = self.nodeDifference(xCoord, R_WRIST, NECK)                              #for pointing right, difference between right wrist and neck
        xLHandHead = self.nodeDifference(xCoord, L_WRIST,NECK)                               #for pointing left, difference between left wrist and neck
        yRHandKnee = self.nodeDifference(yCoord,R_WRIST,R_KNEE)                              #for pointing right and raising right arm, difference between right wrist and right knee
        yLHandKnee = self.nodeDifference(yCoord,L_WRIST,L_KNEE)                              #for pointing left and raising left arm, difference between left wrist and left knee
        yLHipKnee = self.nodeDifference(yCoord,L_HIP,L_KNEE)                                 #for sitting and standing, difference between left hip and left knee
        yLKneeAnkle = self.nodeDifference(yCoord,L_KNEE,L_ANKLE)                             #for sitting and standing, difference between left knee and left ankle
        yRHipKnee = self.nodeDifference(yCoord,R_HIP,R_KNEE)                                 #for sitting and standing, difference between right hip and right knee
        yRKneeAnkle = self.nodeDifference(yCoord,R_KNEE,R_ANKLE)                             #for sitting and standing, difference between right knee and right ankle

        dxR = self.nodeDifference(xCoord,R_ELBOW,R_WRIST)                                    #for waving with right arm, difference between x location of elbow and wrist
        dxL = self.nodeDifference(xCoord,L_ELBOW,L_WRIST)                                    #for waving with left arm, difference between x location of elbow and wrist
        dyR = self.nodeDifference(yCoord,R_ELBOW,R_WRIST)                                    #for waving with right arm, difference between y location of elbow and wrist
        dyL = self.nodeDifference(yCoord,L_ELBOW,L_WRIST)                                    #for waving with left arm, difference between y location of elbow and wrist

        xL = abs(xLHandHead/(maxX))                                                     #x threshold for pointing left
        xR = abs(xRHandHead/(maxX))                                                     #x threshold for pointing right
        yL = abs(yLHandKnee/(maxY))                                                     #y threshold for poining left and raising left hand
        yR = abs(yRHandKnee/(maxY))                                                     #y threshold for pointing right and raising right hand
       # yLeftLeg = abs(yLHipKnee/yLKneeAnkle)                                           #threshold for sitting and standing for left leg
       # yRightLeg = abs(yRHipKnee/yRKneeAnkle)                                          #threshold for sitting and standing for right leg
        pose = abs(maxX/maxY)                                                           #threshold for determining pose, such as lying or sitting/standing

        yLeftThigh = abs(yLHipKnee/yLHeadHip) #instead of maxY
        yRightThigh = abs(yRHipKnee/yRHeadHip) #instead of maxY

        angleR = math.atan2(dyR,dxR)                                                    #used to determine angle for waving for right arm
        angleL = math.atan2(dyL,dxL)                                                    #used to determine angle for waving for left arm
        angleR = math.degrees(angleR)                                                   #converts angle for right arm to degrees
        angleL = math.degrees(angleL)                                                   #converts angle for left arm to degrees
        print("xL:" + str(xL) + "\n")
        print("xR:" + str(xR) + "\n" )
        print("yL:" + str(yL) + "\n" )
        print("yR:" + str(yR) + "\n" )
        print("Pose:" + str(pose) + "\n")
        print("yLeftThigh:" + str(yLeftThigh) + "\n" )
        print("yRightThigh:" + str(yRightThigh) + "\n")
        print("yLHipKnee:" + str(yLHipKnee)+ "\n")
        print("yRHipKnee:" + str(yRHipKnee) + "\n")
        print("angleR:" + str(angleR) + "\n")
        print("angleL:" + str(angleL) + "\n")
        print("MaxX: " + str(maxX) + "\n")
        print("MaxY: " + str(maxY) + "\n")


    #Poses                                                                              #can only be one pose
        activity = self.personPose(pose,yLeftThigh,yRightThigh)
        if(activity == "Laying down"):
            isLaying =1;

    #Gestures                                                                           #can be any combination of gestures
        activity = activity + self.leftArm(xL,yL, angleL, isLaying)
        activity = activity + self.rightArm(xR,yR, angleR, isLaying)

        return activity                                                                 #return string of activities done by person

    def estimation_callback(self,msg):
        i =0
        people_count=0

        for i in msg.persons:
            people_count +=1;

        self.people_size = people_count;
        self.people_list = msg.persons
        # self.people_ids = msg.people

    def process_message(self):
        self.activities = []                                                                 #array of activities per person
        for person in self.people_list:                                                      #for all visible people
            activity = self.process_person(person)                                           #finds activities done by person
            self.activities.append(activity)
       # return activities
       #adds activities to array
        #pub.publish(activities)

    def execute_cb(self, goal):
        #success = False
        result = activity_detection.msg.activityDetectionResult()
        result.activities =[]
        print(self.people_size)
        if(self.people_size>0):
            #print(Estimations.people[0].id)
            #result.personID = self.people_ids[0].id
            #success = True
            self.process_message()
            result.activities=self.activities
        else:
            print('No person found')

        # check that preempt has not been requested by the client
        #if self._as.is_preempt_requested():
        #    rospy.loginfo('%s: Preempted')
        #    self._as.set_preempted()
        #    break

        #if success:
        #rospy.loginfo('Succeeded')
        #print(self.activities)
        self._as.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('activity_detection')
    print("Initialized node")
    server = ActivityDetection_Action(rospy.get_name())
    print("Action ready")
    rospy.spin()
