#!/usr/bin/env python#

#A Flight Status Class
#Implements the methods needed to determine flight status such as battery condition, state convergence, distance from home location
#Most function return a boolean indicator partaining to the query requested. 

PKG = 'fsm' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
import math
from rospy.numpy_msg import numpy_msg
from Utility import *
import copy
import sys
import time
import random
import message_filters
#from message_filters import Subscriber, Cache
from std_msgs.msg import Float64, Bool, Int16
from geometry_msgs.msg import Pose, Point, Quaternion
from rospy.numpy_msg import numpy_msg 
#Import the Listerner and Logger class
from ListenerClass import ListenerClass


class FlightStatusClass():
    """
    Implements the methods needed to determine flight status such as battery condition, state convergence, distance from home location
    """
    def __init__(self,dictionary,queue_size,MinBattVol,safeAltitude,groundlev, throttleThreshold, MaxTime,home,FSM_refreshRate,tolerance):
        print("Initializing Flight Status Object!")
        self.listener                = ListenerClass(queue_size,dictionary) #An instance of class listener used as a subscriber and data logger
        self._minimalBatteryVoltage  = MinBattVol                           #Minimal battery voltage allowed for flights
        self._groundLevel            = groundlev                            #Ground altitude in /world frame of reference
        self._safeAltitude           = safeAltitude                         #Safe operational altitude in /world frame of reference
        self._throttleThreshold      = throttleThreshold                    #Throttle stick (TX) threshold above which FSM would consider pilot permission for takeoff
        self._missionStartTime       = rospy.Time.now().to_sec()            #Mission State Time (in ROS)
        self._missionMaxTime         = MaxTime                              #Alloted time for mission
        self.tolerance               = tolerance                            #Distance tolerance [meters] used for convergence and arrival indication
        self.sleepTime               = FSM_refreshRate                      #For debugging - A time delay when entering each state, set to 0.0 when operational
        self.setTargetPose()                                                #Sets default target pose
        self.setHomePose(home.position, home.orientation)                   #Sets initial home pose
            
    def getMinimalBatteryVoltage(self):
        """
        :return: Minimal Allowed Battery Voltage
        
        Accesor function 
        """
        return self._minimalBatteryVoltage
    
    def getGroundLevel(self):
        """
        :return: Ground level in meters in world frame
        
        Accesor function 
        """
        return self._groundLevel

    def getSafeAltitude(self):
        """
        :return: Flight Status Safe Altitude
        
        Accesor function 
        """
        return self._safeAltitude

    def getThrottleThreshold(self):
        """
        :return: Tx Throttle threshold for considering human pilot intention
        
        Accesor Function 
        """        
        return self._throttleThreshold
    
    def getMissionStartTime(self):
        """
        :return: ROS time object designating when the mission has started
        
        Accesor Function 
        """        
        return self._missionStartTime
    
    def getMissionMaxTime(self):
        """
        :return: ROS time object designating maximal allowed mission duration
        
        Accesor Function 
        """        
        return self._missionMaxTime
    
    def getHomePose(self):
        """
        :return: ROS msg of type "geometry_msgs\Pose.msg" designating the [x,y,z] corrdinates of HOME and an arbitrary quaternion
        
        Accesor Function 
        """        
        return self._homePose

    def getCurrentThrottle(self):
        """
        :return: current Tx throttle level
        
        Accesor Function 
        """        
        return self.listener.ctrlThrottle
     
    def getCurrentBatteryVoltage(self):
        """
        :return: current battery voltage
        
        Accesor Function 
        """        
        return self.listener.batteryVoltage
    
    def getCurrentPoseStamped(self):
        """
        :return: Current (latest) Stamped Pose msg type
        
        Accesor Function 
        """        
        #print self.listener.poseStampedQueue    
        return copy.deepcopy(self.listener.poseStampedQueue[-1])
    
    def getCurrentPose(self):
        """
        :return: Current (latest) Stamped Pose msg type
        
        Accesor Function 
        """        
        return copy.deepcopy(self.getCurrentPoseStamped().pose)
    
    def getCurrentState(self,str_state):
        """
        :param: str_state: string of the state to be returned , either 'x','y','z'
        :return: Current (latest) position attribute, either 'x','y','z'
        
        Accesor Function 
        """
        return copy.deepcopy(getattr(self.getCurrentPose().position,str_state)   )
    
    def getCurrentAltitude(self):
        """
        :return: Current (latest) altitude 'z' 
        
        Accesor Function 
        """        
        return self.getCurrentState('z')
    
    def getMissionDuration(self):
        """
        :return: Mission duration in seconds thus far
        
        Accesor Function for mission duration thus far [seconds]
        """
        return rospy.Time.now().to_sec()-self.getMissionStartTime()
    
    def getTargetPose(self):
        """
        :return: pose
        
        Accesor Function to get the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        return copy.deepcopy(self._targetPose)
        
    def setTargetPose(self,position = Point(0.0,0.0,2.0) ,orientation = Quaternion(0.0,0.0,0.0,1.0) ):
        """
        :return: void
        
        Accesor Function to set the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        
        #Ensure the target pose altitude is above safeAltitude
        position.z = position.z if position.z > self.getSafeAltitude() else self.getSafeAltitude()
        self._targetPose = Pose(copy.copy(position),copy.copy(orientation))
        return 
    
    def getHomePose(self):
        """
        :return: pose
        
        Accesor Function to get the current home pose of the vehicle 
        """
        return self._homePose 
    
    def setHomePose(self,position = Point(0.0,0.0,2.0) ,orientation = Quaternion(0.0,0.0,0.0,1.0) ):
        """
        :return: void
        
        Accesor Function to set the current target pose of the vehicle (to where the controller is aiming to drive)
        """
        self._homePose = Pose(copy.deepcopy(position),copy.deepcopy(orientation))
        return 
    
    def IsThrottleUp(self):
        """
        :return: A boolean indicating whether TX throttle level is above threshold
        
        Function indicating whether TX throttle level is above threshold
        """
        return self.listener.ctrlThrottle>self._throttleThreshold    
    
    def IsTimeExceeded(self):
        """
        :return: A boolean indicating whether mission duration has exceeded alloted time
        
        Function indicated whether alloted time for mission has be exceeded
        """
        return self.getMissionDuration()>self.getMissionMaxTime()
        
    def IsHome(self):
        """
        :return: A boolean indicating whether vehicle is within a predefined threshold of the distance home (2D - planar)
        
        Function indicates if vehicle is in home coordinates
        """
        return self.DistanceToHome(2)<self.tolerance
    
    def ErrorConverge(self,str_attribute):
        """
        :param: str_attribute: string of the state to be returned , either 'x','y','z'
        :return: A boolean indicating whether state has converged 
        
        Utility function to determine if error of controller has converged
        """        
        #Distance to Target Pose
        #print "Check if Error has converged: " 
        #print "\nStart pose: " ,      self.getCurrentPoseStamped().pose
        #print "\nTarget pose: " ,      self.getTargetPose()
       
        
        #Controller Errors
        e_mean, e_var = self.listener.runningStatError[self.listener.dictionary[str_attribute]].Mean_Variance()
        e_d_mean, e_d_var = self.listener.runningStatError_d[self.listener.dictionary[str_attribute]].Mean_Variance()
        print "\nError Mean", e_mean
        print "Error Var", e_var       
     
        print "\nError Derivative Mean", e_d_mean
        print "Error Derivative Var" , e_d_var

        bool_error   =  abs(e_mean) < self.tolerance and e_var< math.pow(self.tolerance,2)
        bool_error_d =  abs(e_d_mean)  < self.tolerance and e_d_var< math.pow(self.tolerance,2)
        if bool_error and bool_error_d:
            print "\nError in " ,str_attribute ,"Converged"
            return True
        else :
            print "\nError in " ,str_attribute ,"Did not Converge"
            return False
    
    def PositionErrorConverge(self):
        """
        :param: void
        
        :return: A boolean indicating whether position error has converged and velocity is ~zero (vehicle is hovering)
        """
        distance_to_target = self.DistanceToTarget(3)
        print "\nDistance to TARGET: " , distance_to_target
        
        bool    = abs(distance_to_target)< self.tolerance #Close to target
        if not bool:
            return False
        else:
            for str in 'xyz':
                bool *= self.ErrorConverge(str)
            return bool

    def ErrorDiverge(self,str_attribute):
        """
        :param: str_attribute: string of the state to be returned , either 'x','y','z'
        :return: A boolean indicating whether state is in the process of diverging 
        
        Utility function to determine if error of controller is divering / unstable
        """        
        e_d_mean, e_d_var = self.listener.runningStatError_d[self.listener.dictionary[str_attribute]].Mean_Variance()
        #bool_error   = self.listener.runningStatError[self.listener.dictionary[str_attribute]].Mean()    < 1       
        #print "e_d_mean", e_d_mean
        bool_error_d = abs(e_d_mean)  > 10*self.tolerance       
        if bool_error_d :
            print "Error derivative in " ,str_attribute ,"Diverged"            
            return True
            
        else :
##            print "Error derivative in " ,str_attribute ,"Did not Diverge"            
            return False

    def PositionErrorDiverge(self):
        """
        :return: A boolean indicating whether ANY is in the process of diverging 
        
        Utility function to determine if any errors / states are divering / unstable
        """                
        bool = False 
        for str in 'xyz':
            bool *= not self.ErrorDiverge(str)
        return bool

    def DistanceToHome(self,dim):
        """
        :return: A float representing the 2D (x-y planar) Euclidean distance of the vehicle from home
    
        """        
        dist = Distance('Euclidean',
                        PoseMsg2NumpyArrayPosition(self.getCurrentPose() ),
                        PoseMsg2NumpyArrayPosition(self.getHomePose()    ),
                        dim)
        return dist


    def DistanceToTarget(self,dim):
        """
        :return: A float representing the 2D (x-y planar) Euclidean distance of the vehicle from target pose
    
        """        
        dist = Distance('Euclidean',
                        PoseMsg2NumpyArrayPosition(self.getCurrentPose() ),
                        PoseMsg2NumpyArrayPosition(self.getTargetPose()    ),
                        dim)
        return dist
    
    
    def VoltageNeededToGetHome(self):
        """
        :return: A float representing the estimated voltage needed to return home from present location
        
        Utility function computing/estimating the needed voltage to get home from present location
        Can be later implemented as a table lookup [Euclidean Dist, Voltage] or an energy mapping of a trajectory generated by a motion planner called
        """        
        #Presently implements an arbitrary scaling from distance to voltage 
##        CurrentStampedPose = self.getCurrentPoseStamped()
        dist = self.DistanceToHome(2)       
        print("Vehicle is a distance of %s meters away from home " %dist)
        return dist*0.001 #<<<--- !!!
        
        
    def IsBatteryOK(self):
        """
        :return: A boolean indicating whether battery status is ok
        
        Function indicating whether battery status is ok (sufficient voltage to continue mission) 
        taking into account minimal safe voltage level,distance from home coordinates and present voltage
        """
        #Compute whether battery level is sufficient based on present voltage level, battery predefined threshold distance to home
##        print ('\n\nCurrent Battery Voltage' , self.listener.batteryVoltage )        
##        print ('Minimal Batt Voltage allowed :' , self.minimalBatteryVoltage )
##        print ('Battery to get HOME:' , self.VoltageNeededToGetHome() )
        if self.getCurrentBatteryVoltage() < self.getMinimalBatteryVoltage() + self.VoltageNeededToGetHome() : 
            sys.stdout.write('\rNot Enough Battery...')            
            return False
        else :
            print ('We are good, Enough Battery')
            return True
    
##    def window(self,size):
##        return numpy.ones(size)/float(size) #Uniform weights
    
   

 