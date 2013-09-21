#!/usr/bin/env python#

#A Flight Status Class
#Implements the methods needed to determine flight status such as battery condition, state convergence, distance from home location
#Most function return a boolean indicator partaining to the query requested. 


PKG = 'rospy_tutorials' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
from rospy.numpy_msg import numpy_msg
from Utility import *
import time
import random
import message_filters
#from message_filters import Subscriber, Cache
from std_msgs.msg import Float64, Bool, Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from rospy.numpy_msg import numpy_msg 

from ListenerClass import ListenerClass


class FlightStatusClass():
    def __init__(self,MinBattVol,safeAltitude,groundlev, throttleThreshold, MaxTime,home,FSM_refreshRate):
        print("Initializing Flight Status Object!")
        self.listener               = ListenerClass()
        self.minimalBatteryVoltage  = MinBattVol
        self.groundLevel            = groundlev
        self.safeAltitude           = safeAltitude
        self.missionStartTime       = rospy.Time.now().to_sec()
        self.missionMaxTime         = MaxTime
        self.throttleThreshold      = throttleThreshold
        self.homeCoordinates        = home #geometry_msgs/Point
        self.sleepTime              = FSM_refreshRate #When entering states, how long to sleep - slows down the FSM, used for debugging
    
    
    def getCurrentPoseStamped(self):
        #Accesor Function 
        #print self.listener.poseStampedQueue    
        return self.listener.poseStampedQueue[-1]    
    
    def getAltitude(self):
        #Accesor Function 
        return self.getCurrentPoseStamped().pose.position.z
    
    def getMissionDuration(self):
        return rospy.Time.now().to_sec()-self.missionStartTime
    
    def IsTimeExceeded(self):
        return self.getMissionDuration()>self.missionMaxTime
        
    
    def ErrorConverge(self,i):
        #Compute whether state i has converged 
##        history_vector = self.listener.poseStampedQueue 
##        print(history_vector)
##        N   	       = history_vector.size
##        average        = numpy.convolve(history_vector,self.window(N),'valid') #Uniform window
        if random.uniform(0,1)>0 : 
         return True
        else :
         return False

    def ErrorDiverge(self,i):
        #Compute whether state i is diverging
       # history_vector = self.listener.poseStampedQueue # Access to the i-th row of the arrayself.poseStampedQueue
##        N   	       = history_vector.size
##        average = []
##        for i in xrange(1,N):
##            average.append(numpy.convolve(history_vector,self.window(N),'valid')) #Uniform window
##        
##        variance = numpy.var(average)

##        if variance > 0.1:
##            return True
##        else :
##            return False
##        
        if random.uniform(0,1)>1.0 : 
         return True
        else :
         return False

    def AnyErrorDiverge(self):
        #Compute whether any state is diverging
##        bool = False 
##        for i in xrange(0,2):
##            bool *= not self.ErrorDiverge(i)
##        return not bool
##    
        if random.uniform(0,1)>1.0 : 
            return True
        else :
            return False

    
    
    def VoltageNeededToGetHome(self):
        #Arbitrary scaling from distance to voltage - obviously should be properly mapped
        #Should be the output from a service call to a motion planner with path energy estimated cost
        CurrentStampedPose = self.getCurrentPoseStamped()
        dist = Distance('Euclidean',
                        PoseMsg2NumpyArrayPosition(CurrentStampedPose.pose),
                        PointMsg2NumpyArrayPosition(self.homeCoordinates),
                        3)        
        print("Vehicle is a distance of %s meters away from home " % dist)
        return 0.1*dist 
        
        
    def IsBatteryOK(self):
        #Compute whether battery level is sufficient based on present voltage level, battery predefined threshold distance to home
##        print ('\n\nCurrent Battery Voltage' , self.listener.batteryVoltage )        
##        print ('Minimal Batt Voltage allowed :' , self.minimalBatteryVoltage )
##        print ('Battery to get HOME:' , self.VoltageNeededToGetHome() )
        rospy.sleep(2.0)
        if self.listener.batteryVoltage < self.minimalBatteryVoltage + self.VoltageNeededToGetHome() : 
            print ('Not Enough Battery')
            return False
        else :
            print ('We are good, Enough Battery')
            return True
    
    def window(self,size):
        return numpy.ones(size)/float(size) #Uniform weights
    
   

 