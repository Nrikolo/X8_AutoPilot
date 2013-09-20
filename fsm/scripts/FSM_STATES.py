#!/usr/bin/env python
#General Imports
import roslib; #roslib.load_manifest('smach_tutorials')
roslib.load_manifest('fsm')
import rospy
import time
import random
import math 

import smach
import smach_ros
#For dealing with msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# The Listener object used in the FSM
#from ListenerClass import ListenerClass
from beginner_tutorials.srv import *



# define state MANUAL
class MANUAL(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Finish',
                                             'TOAUTONOMOUS'],
                                   input_keys=['manual_Home'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state MANUAL')
        rospy.sleep(self.flightStatus.sleepTime)
        while True:
            #rospy.loginfo(" I heard  %s  ",  self.flightStatus.homeCoordinates)
            if self.flightStatus.listener.AutoPilotSwitch == True  and self.flightStatus.listener.MissionGoSwitch == True:
                print ("Both Switches are ON - Autopilot turned on, PC has control!")                 
                return 'TOAUTONOMOUS'
                
            if self.flightStatus.IsTimeExceeded() : 
                print ("Mission Duration Exceeded - Finish") 
                return 'Finish'

class AUTONOMOUS_INIT(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self,outcomes=['ToIdle',
                                            'ToHover',
                                            'ToLand',
                                            'Failure'],
                                  input_keys=['AutoInit_Home'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state AUTONOMOUS_INIT')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.listener.AutoPilotSwitch == True : # and self.flightStatus.ctrlThrottle< userdata.AutoInit_throttleThreshold:
            z = self.flightStatus.getAltitude()
            if z > self.flightStatus.safeAltitude: #Safe
                print ("Vehicle above minimal safe altitude - goto HOVER")                 
                return 'ToHover'
            elif z < self.flightStatus.groundLevel: #On the GROUND
                print ("Vehicle seems to be still on the ground - goto IDLE")                                 
                return 'ToIdle'
            else :
                print ("Vehicle in intermediate altitude - goto LAND")                                 
                return 'ToLand' #Intermediate altitude - LAND!
        else:
            return 'Failure'


# define state TAKEOFF
class TAKEOFF(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                            'Maintain',
                                            'Aborted_NoBatt',
                                            'Aborted_Diverge'],
                                   input_keys=['takeoff_Home'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state TAKEOFF')
        rospy.sleep(self.flightStatus.sleepTime)        
               
        if self.flightStatus.listener.AutoPilotSwitch == False or  self.flightStatus.AnyErrorDiverge()==True:
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            return 'Aborted_Diverge'
        elif self.flightStatus.listener.MissionGoSwitch == False or self.flightStatus.IsBatteryOK()==False :
            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
            return 'Aborted_NoBatt'
        if self.flightStatus.ErrorConverge(2)==True: 
            print ("Takeoff complete and succeful - goto HOVER")
            return 'Success'
        print ("TakingOff...")#Later should be mapped to GOHOME state
        return 'Maintain'


# define state HOVER
class HOVER(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['MissionDone',
                                            'Maintain',
                                            'Aborted_NoBatt',
                                            'Aborted_Diverge'],
                                   input_keys=['hover_Home'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state HOVER')
        rospy.sleep(self.flightStatus.sleepTime)
        z_ref = self.flightStatus.getAltitude() #use present altitude as ref
      
        if self.flightStatus.AnyErrorDiverge():
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            return 'Aborted_Diverge' #->Manual!
        if not self.flightStatus.IsBatteryOK():
            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
            return 'Aborted_NoBatt' #->Vehicle should LAND
        if self.flightStatus.IsTimeExceeded() : #Mission Duration Reached
            print ("Mission Duration Exceeded - Finish") 
            return 'MissionDone'
        else : #Maintain hover
            print("Hovering....")
            return 'Maintain'
   
    
# define state LAND
class LAND(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                            'Maintain',
                                            'Failure'],
                                   input_keys=['land_Home'])
        self.flightStatus = flightStatus
     
     def execute(self, userdata):
        rospy.loginfo('Executing state LAND')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.AnyErrorDiverge():
            print ("Vehicle is unstable - goto MANUAL")                 
            return 'Failure' #->Manual!
        if self.flightStatus.ErrorConverge(2,self.flightStatus.groundLevel):    
           print ("Vehicle has landed - goto IDLE")                 
           return 'Success' #->Idle
        
        print ("Landing...")                  
        return 'Maintain' #Remain in Land!
 
               
# define state IDLE
class IDLE(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Finish',
                                             'Start',
                                            'Maintain'],
                                   input_keys=['idle_Home'])
        self.flightStatus = flightStatus
     
     def execute(self, userdata):
        rospy.loginfo('Executing state IDLE')
        rospy.sleep(self.flightStatus.sleepTime)
        #Should deal with controllers?
        if self.flightStatus.getMissionDuration()>rospy.Duration(100).to_sec():
           print ("Waited in idle for too long - goto to MANUAL") 
           return 'Finish' #- to manual
        elif self.flightStatus.listener.ctrlThrottle > self.flightStatus.throttleThreshold: 
           print ("Seems like pilot wants to take off - goto TAKEOFF") 
           return 'Start'  #- to takeoff
        else:
            print("Idle...")
            return 'Maintain'



# define state InitContoller
class ControllerInit(smach.State):
    def __init__(self,flightStatus,controlManagement , str_ParentStateName):
        smach.State.__init__(self, outcomes=['Success','Failure'])
        self.flightStatus        = flightStatus
        self.controlManagement   = controlManagement
        self.str_ParentStateName = str_ParentStateName #Used to indicate which controller should be turned ON

    def execute(self,userdata):
        rospy.loginfo('Executing state ControllerInit in %s' , self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        # Create a service request structure
        Service_in = ControllerRequest()         
        # Designated whether the controller should turn ON or OFF
        Service_in.ON_OFF = True
        # Call a function that generates a trajectory for the controller to follow
        if self.str_ParentStateName is 'IDLE':
            print 'creating a StampedPose to be used as a constant ref signal for the controller'
            StampedPose                  = self.flightStatus.getCurrentPoseStamped() 
            #StampedPose                  = PoseStamped() #Construct a StampedPose MSG
            Service_in.trajectory.append(StampedPose) 
        elif self.str_ParentStateName is 'HOVER':
            print 'creating a StampedPose to be used as a constant ref signal for the controller'
            StampedPose                  = self.flightStatus.getCurrentPoseStamped() 
            #StampedPose                  = PoseStamped() #Construct a StampedPose MSG
            StampedPose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
            Service_in.trajectory.append(StampedPose) 
        else: #TAKEOFF or LAND    
            if self.str_ParentStateName is 'TAKEOFF':
                z_final = 1.05 * self.flightStatus.safeAltitude
                print 'Generating trajectory for TAKEOFF'
            elif self.str_ParentStateName is 'LAND':
                z_final = self.flightStatus.groundLevel
                print 'Generating trajectory for LAND'
            v_max = 2
            delay = 2
            Service_in.trajectory = getTrajectoryAltitudeProfile(self.flightStatus.getCurrentPoseStamped(),
                                                             z_final,
                                                             v_max,
                                                             delay)
        
        # Designated whether there are controller gains to be adjusted
        Service_in.input_gain_flag = False
        # Controller PID gains (regarded only if flag is TRUE)
        Service_in.gains = [1.0,2.0,3.0]
        
        # Send service request
        if self.controlManagement.ControllerClient(Service_in):
            print("Controller turned ON")
            return 'Success'
        else:
            print("Controller FAILED to turn on")
            return 'Failure'


# define state TERMINATE
class TERMINATE(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Done'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state TERMINATE')
        #Turn off ALL controllers
        print ('All Controllers turned off - we are DONE') 
        return 'Done'


def getTrajectoryAltitudeProfile(currentPose, z_final, v_max, delay):
    print 'Genereting an altitude profile'
    #Ref signal frequency
    frequency         = 100 #[Hz]
    #Time duration of sigamoid function
    sigamoid_duration = 10 #sec
    #Number of samples
    L      = sigamoid_duration * frequency 
    #Use current position [x,y] constant and current z as lower asymptote of sigamoid
    x      = currentPose.pose.position.x
    y      = currentPose.pose.position.y
    z_init = currentPose.pose.position.z
    quat   = Quaternion(0.0, 0.0, 0.0, 1.0)
    #Loop to populate trajectory with StampedPoses instances
    trajectory = [] # z = z_init + (z_final - z_init)./(1+exp(-v_max*(t-delay-tNOW))));    
    TrajectoryStartTime = rospy.Time.now()
    for i in range(0,L):
        StampedPose                 = PoseStamped() #Construct a StampedPose MSG
        StampedPose.header.frame_id = "/Body" #Frame of ref that the trajectory is formualted in
        t                           = TrajectoryStartTime + rospy.Duration(1/frequency)        #create a time instance
        z                           = z_init + (z_final - z_init)/(1+math.exp(-v_max*(t.to_sec()-delay-TrajectoryStartTime.to_sec() )));    #compute its height
        #Populate the StampedPose object
        StampedPose.header.stamp    = t
        StampedPose.pose            = Pose(Point(x, y, z),quat) #The pose
        # Append the time staped pose to the trajectory to follow
        trajectory.append(StampedPose)
    return trajectory