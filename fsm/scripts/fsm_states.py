#!/usr/bin/env python
#General Imports
import roslib; #roslib.load_manifest('smach_tutorials')
roslib.load_manifest('fsm')
import rospy
import time
import random
import math 
import numpy
import smach
import smach_ros
#For dealing with msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# The Listener object used in the FSM
#from ListenerClass import ListenerClass
#from beginner_tutorials.srv import *
from Utility import *
from quadrotor_input.srv import *


# define state MANUAL
class MANUAL(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Finish',
                                            'Monitor',
                                            'TOAUTONOMOUS'])
##                                   input_keys=['manual_mission_stage_in'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
##        rospy.loginfo('Executing state MANUAL')
        rospy.sleep(self.flightStatus.sleepTime)
        #rospy.loginfo(" I heard  %s  ",  self.flightStatus.homeCoordinates)
        if ( self.flightStatus.IsBatteryOK() ) and ( self.flightStatus.listener.AutoPilotSwitch == True  ) :
            print ("AutoPilot switch is ON, there is enough battery ---->>> Transfering control to PC ")                 
            return 'TOAUTONOMOUS'
             
        if self.flightStatus.IsTimeExceeded() : 
            print ("Mission Duration Exceeded - Finish") 
            return 'Finish'
        else:
            return 'Monitor'

class AUTONOMOUS_INIT(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self,outcomes=['ToIdle',
                                            'ToHover',
                                            'ToLand',
                                            'Failure'])
##                                  input_keys=['AutoInit_mission_stage_in'],
##                                  output_keys=['AutoInit_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state AUTONOMOUS_INIT')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.IsBatteryOK() and self.flightStatus.listener.AutoPilotSwitch == True : 
            z = self.flightStatus.getCurrentAltitude()
            if z - self.flightStatus.getSafeAltitude() > self.flightStatus.tolerance: #Safe
                print ("Vehicle above minimal safe altitude - goto HOVER")                 
                return 'ToHover'
            elif z - self.flightStatus.getGroundLevel() < self.flightStatus.tolerance: #On the GROUND
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
                                             'Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                    input_keys  = ['TakeOff_mission_stage_in'],
##                                    output_keys = ['TakeOff_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state TAKEOFF')
        rospy.sleep(self.flightStatus.sleepTime)        
        if ( self.flightStatus.listener.AutoPilotSwitch == False ) or  ( self.flightStatus.PositionErrorDiverge()== True ):
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            return 'Aborted_Diverge'
        elif (self.flightStatus.listener.MissionGoSwitch == False ) or ( self.flightStatus.IsBatteryOK()== False ):
            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
            return 'Aborted_NoBatt'
        if self.flightStatus.PositionErrorConverge()==True: 
            print ("Takeoff complete and succeful - goto HOVER")
            return 'Success'
        print ("TakingOff...")#Later should be mapped to GOHOME state        
        return 'Maintain'

            
# define state HOVER
class HOVER(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain',
                                            'GoHome',
                                            'FollowTraj'])
##                                    input_keys  = ['Hover_mission_stage_in'],
##                                    output_keys = ['Hover_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state HOVER')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge() or (self.flightStatus.listener.AutoPilotSwitch is False) :
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            return 'Aborted_Diverge' #->Manual!
        if ( not self.flightStatus.IsBatteryOK() ) or (self.flightStatus.listener.MissionGoSwitch is False)  or self.flightStatus.IsTimeExceeded() : 
            print ("Either pilot wants vehicle to come home, duration exceeded or there is no Battery")#Later should be mapped to GOHOME state
            #print "Battery Voltage Level: " ,self.flightStatus.getCurrentBatteryVoltage()
            #print "MissionGo Switch: " ,self.flightStatus.listener.MissionGoSwitch                   
            if self.flightStatus.IsHome():
                return 'Aborted_NoBatt' #->Vehicle should LAND
            else: 
                return 'GoHome' #->Vehicle should return home
        print "self.flightStatus.DistanceToTarget(3)", self.flightStatus.DistanceToTarget(3)        
        if self.flightStatus.DistanceToTarget(3) > 10 * self.flightStatus.tolerance:
            print("Far away from target, should generate a trajectry to go there")
            return 'FollowTraj'
        print("Hovering....")
        return 'Maintain'
            
                
# define state LAND
class LAND(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                            'Failure',
                                            'Maintain'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
##        rospy.loginfo('Executing state LAND')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge() or self.flightStatus.listener.AutoPilotSwitch==False:
            print ("Vehicle is unstable - goto MANUAL")                 
            return 'Failure' #->Manual!
        if self.flightStatus.PositionErrorConverge():    
            print ("Vehicle has landed - goto IDLE")                 
            return 'Success' #->Idle
        print ("Landing...")                  
        return 'Maintain' #Remain in Land!
     
               
# define state IDLE
class IDLE(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Finish',
                                             'Start',
                                            'Maintain'])
##                                   input_keys  = ['Idle_mission_stage_in'],
##                                    output_keys = ['Idle_mission_stage_out'])
        self.flightStatus = flightStatus
                
     def execute(self, userdata):
##        rospy.loginfo('Executing state IDLE')
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.listener.AutoPilotSwitch == False or  not self.flightStatus.IsBatteryOK():
            print ('All Controllers turned off - we are DONE') 
            #Waited for a while in idle or one of the switches is OFF
            print ("AutoPilot is OFF  --->>> goto MANUAL") 
            return 'Finish' #- to manual
        elif self.flightStatus.IsThrottleUp() and self.flightStatus.IsBatteryOK() and self.flightStatus.listener.MissionGoSwitch == True : 
            #Throttle is up and there is enough battery
            print ("Seems like pilot wants to take off and there's enough battery --->>> goto TAKEOFF") 
            return 'Start'  #- to takeoff
        print("Idle...")    
        return 'Maintain'

# define state FOLLOWTRAJECTORY  (This state should be is a template for GoHome or any other follow traj, the only difference is what is the trajectory to follow)
class FOLLOW_TRAJECTORY(smach.State):
     def __init__(self,flightStatus,str_ParentStateName):
        smach.State.__init__(self, outcomes=['Arrived',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                   input_keys  = ['TrajFol_mission_stage_in'],
##                                   output_keys = ['TrajFol_mission_stage_out'])
        self.flightStatus = flightStatus
        self.str_ParentStateName = str_ParentStateName
                
     def execute(self, userdata):
##        rospy.loginfo('Executing state FOLLOW_TRAJECTORY inside %s', self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        #Should add that is in follow trajectory and target pose is homepose, then maintain...
        if self.flightStatus.PositionErrorDiverge() or self.flightStatus.listener.AutoPilotSwitch == False:
            print ("Either pilot wants control back or vehicle is unstable --->>> goto MANUAL")                 
            return 'Aborted_Diverge' #--->>>Manual!
                
        if self.flightStatus.PositionErrorConverge() : #Regardless of parent container, if arrived at destination, should goto HOVER
            print ("Seems like vehicle arrived at destination --->>> goto HOVER") 
            return 'Arrived'

        if self.flightStatus.listener.MissionGoSwitch == False or not self.flightStatus.IsBatteryOK() or self.flightStatus.IsTimeExceeded() :                 
            print ("Either pilot wants vehicle to return on, or no battery or no time left for mission...")
            if self.str_ParentStateName is 'GO_HOME' :    
                    print ("Vehicle returning home...")
                    return 'Maintain' #->Vehicle should continue going home
            else: 
                    print ("Vehicle following some trajectory, but should be returning home...first goto -->>HOVER")
                    return 'Arrived' #->Vehicle should go to HOVER
        
        print("Following Trajectory")
        return 'Maintain'
            

# define state InitContoller
class CONTROLLER_INIT(smach.State):
    def __init__(self, flightStatus, controlManagement , str_ParentStateName):
        smach.State.__init__(self, outcomes=['Success','Failure'])
        self.flightStatus        = flightStatus
        self.controlManagement   = controlManagement
        self.str_ParentStateName = str_ParentStateName #Used to indicate which controller should be turned ON
        self.dict = {'True': 'ON', 'False':'OFF'};
        #self.stateDictionary = {'IDLE': 1, 'HOVER':2,'LAND':3,'TAKEOFF':4,'GO_HOME':5};

    def execute(self,userdata):
##        rospy.loginfo('Executing state ControllerInit in %s' , self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        # Create a service client
        Service_in = CommandControllerRequest()                 
        # Designated whether there are controller gains to be adjusted (Default)
        #Service_in.input_gain_flag = True
        # Controller PID gains (regarded only if flag is TRUE, Default)
        #Service_in.gains = [1.0,2.0,3.0] 
        # Default  - Controller should turn ON
        Service_in.running = True
                
        if self.str_ParentStateName is 'IDLE':
##            print "SwitchCase IDLE"
            if self.flightStatus.listener.AutoPilotSwitch == False or self.flightStatus.listener.MissionGoSwitch == False or not self.flightStatus.IsBatteryOK():
                print ('All Controllers should be turned off...') 
                # Designated that the controller should turn OFF
                Service_in.running = False
            else:
                print("Getting ready to start mission...")
                print ("Creating a TargetPose to be used as a constant ref signal for the controller")
                self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position)
                self.flightStatus._targetPose.position.z = self.flightStatus.getCurrentAltitude()
            
        else:
            for case in switch(self.str_ParentStateName):
                               
                if case('HOVER'):
##                    print "SwitchCase HOVER"
##                    print("Starting Controller for HOVER")
                    print ("Creating a StampedPose to be used as a constant ref signal for the controller")
                    self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position)                
                    break
##                Service_in.path.poses.append(self.flightStatus.getCurrentPoseStamped()) 
                    
                if case('LAND'):
##                    print "SwitchCase LAND"
                    self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position)                
                    self.flightStatus._targetPose.position.z = self.flightStatus.getGroundLevel()
                    print 'Generating trajectory for LAND'
                    break

                if case('TAKEOFF'):
##                    print "SwitchCase TAKEOFF"
                    self.flightStatus.setTargetPose(self.flightStatus.getCurrentPose().position)                
                    self.flightStatus._targetPose.position.z = self.flightStatus.getSafeAltitude() + 0.1 #MOdify the z value of the private targetPose attribute
                    print 'Generating trajectory for TAKEOFF'
                    break
                
                if case('GO_HOME'):
##                    print "SwitchCase GOHOME"
                    self.flightStatus.setTargetPose(self.flightStatus.getHomePose().position)                
##                    print 'GO_HOME'
                    break
        
##        print "Prior to generating a trajectory"
##        print "Current:" , self.flightStatus.getCurrentPose()
##        print "Target:", self.flightStatus.getTargetPose()
        # Call a function that generates a trajectory for the controller to follow - - >>>> SHOULD BE A SERVICE PROVIDOR            
        Service_in.path.poses = getTrajectory(self.flightStatus.getCurrentPose(),
                                              self.flightStatus.getTargetPose(),
                                              self.flightStatus.tolerance)
        
        #Service_in.path.poses = getTrajectoryAltitudeProfile(self.flightStatus.getCurrentPoseStamped(),
        #                                                     self.flightStatus.getTargetPose(),
        #                                                     self.flightStatus.tolerance)
        
        # Send service request
        if self.controlManagement.ControllerClient(Service_in):
            print("Controller SUCCEEDED to turn " + self.dict[str(Service_in.running)] )
            return 'Success'
        else:
            print("Controller FAILED to turn " + self.dict[str(Service_in.running)])
            return 'Failure'

