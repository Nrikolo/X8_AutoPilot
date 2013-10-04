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
from beginner_tutorials.srv import *
from Utility import *



# define state MANUAL
class MANUAL(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Finish',
                                            'Monitor',
                                            'TOAUTONOMOUS'])
##                                   input_keys=['manual_mission_stage_in'])
        self.flightStatus = flightStatus

     def execute(self, userdata):
        rospy.loginfo('Executing state MANUAL')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)
        #rospy.loginfo(" I heard  %s  ",  self.flightStatus.homeCoordinates)
        if ( self.flightStatus.IsBatteryOK() ) and ( self.flightStatus.listener.AutoPilotSwitch == True ) and ( self.flightStatus.mission_stage is 'OUTBOUND' ):
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
        rospy.loginfo('Executing state AUTONOMOUS_INIT')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.IsBatteryOK() and self.flightStatus.listener.AutoPilotSwitch == True : 
            #??? and self.flightStatus.ctrlThrottle< userdata.AutoInit_throttleThreshold:
            z = self.flightStatus.getCurrentAltitude()
            if z > self.flightStatus.safeAltitude: #Safe
                print ("Vehicle above minimal safe altitude - goto HOVER")                 
                return 'ToHover'
            elif z < self.flightStatus.groundLevel: #On the GROUND
                print ("Vehicle seems to be still on the ground - goto IDLE")                                 
                return 'ToIdle'
            else :
                print ("Vehicle in intermediate altitude - goto LAND") 
                self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to TAKEOFF once LANDED
                return 'ToLand' #Intermediate altitude - LAND!
        else:
            self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to restart mission one in MANUAL
##            print ("Mission Stage coming out: %s " %userdata.AutoInit_mission_stage_out)
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
        rospy.loginfo('Executing state TAKEOFF')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)        
        if ( self.flightStatus.listener.AutoPilotSwitch == False ) or  ( self.flightStatus.PositionErrorDiverge()== True ):
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to goto Autonomous Menifold
            return 'Aborted_Diverge'
        elif (self.flightStatus.listener.MissionGoSwitch == False ) or ( self.flightStatus.IsBatteryOK()== False ) or (self.flightStatus.mission_stage is 'INBOUND'):
            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
            self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to TAKEOFF once LANDED
            return 'Aborted_NoBatt'
        if self.flightStatus.ErrorConverge('z')==True: 
            print ("Takeoff complete and succeful - goto HOVER")
            return 'Success'
        print ("TakingOff...")#Later should be mapped to GOHOME state        
##        print ("Mission Stage coming out: %s " %userdata.TakeOff_mission_stage_out)
        return 'Maintain'

            
# define state HOVER
class HOVER(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['MissionDone',
                                            'Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                    input_keys  = ['Hover_mission_stage_in'],
##                                    output_keys = ['Hover_mission_stage_out'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
        rospy.loginfo('Executing state HOVER')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)
        z_ref = self.flightStatus.getCurrentAltitude() #use present altitude as ref
        if self.flightStatus.PositionErrorDiverge():
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to goto Autonomous Menifold
            return 'Aborted_Diverge' #->Manual!
        if ( not self.flightStatus.IsBatteryOK() ) or (self.flightStatus.listener.MissionGoSwitch is False) or (self.flightStatus.mission_stage is 'INBOUND'):
##            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
##            print "battery level: " ,self.flightStatus.getCurrentBatteryVoltage()
##            print "mission go switch: " ,self.flightStatus.listener.MissionGoSwitch             
##            print self.flightStatus.mission_stage
            self.flightStatus.mission_stage = 'INBOUND' #Should not re attampt to TAKEOFF once LANDED
            return 'Aborted_NoBatt' #->Vehicle should LAND
        if self.flightStatus.IsTimeExceeded() : #Mission Duration Reached
            print ("Mission Duration Exceeded - Finish") 
            return 'MissionDone'
        print("Hovering....")
##        print ("Mission Stage coming out: %s " %userdata.Hover_mission_stage_out)
        return 'Maintain'
            
                
# define state LAND
class LAND(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Success',
                                            'Failure',
                                            'Maintain'])
        self.flightStatus = flightStatus
        
     def execute(self, userdata):
        rospy.loginfo('Executing state LAND')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        self.flightStatus.mission_stage = 'INBOUND' #Vehicle is LANDING, should not attempt to take off once landed
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge() :
            print ("Vehicle is unstable - goto MANUAL")                 
            return 'Failure' #->Manual!
        if self.flightStatus.ErrorConverge('z'):    
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
        rospy.loginfo('Executing state IDLE')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.listener.AutoPilotSwitch == False or self.flightStatus.listener.MissionGoSwitch == False or (self.flightStatus.mission_stage is 'INBOUND') or  not self.flightStatus.IsBatteryOK() :
            print ('All Controllers turned off - we are DONE') 
            #Waited for a while in idle or one of the switches is OFF
            print ("AutoPilot if OFF or MissionGo is OFF  --->>> goto MANUAL") 
            self.flightStatus.mission_stage =  'INBOUND'
            return 'Finish' #- to manual
        elif self.flightStatus.listener.ctrlThrottle > self.flightStatus.throttleThreshold and self.flightStatus.IsBatteryOK() and (self.flightStatus.mission_stage is 'OUTBOUND'): 
            #Throttle is up and there is enough battery
            print ("Seems like pilot wants to take off and there's enough battery --->>> goto TAKEOFF") 
            return 'Start'  #- to takeoff
        print("Idle...")    
##        print ("Mission Stage coming out: %s " %userdata.Idle_mission_stage_out)
        return 'Maintain'

# define state FOLLOWTRAJECTORY  (This state should be is a template for GoHome or any other follow traj, the only difference is what is the trajectory to follow)
class FOLLOW_TRAJECTORY(smach.State):
     def __init__(self,flightStatus):
        smach.State.__init__(self, outcomes=['Arrived',
                                            'Aborted_NoBatt',
                                            'Aborted_Diverge',
                                            'Maintain'])
##                                   input_keys  = ['TrajFol_mission_stage_in'],
##                                    output_keys = ['TrajFol_mission_stage_out'])
        self.flightStatus = flightStatus
                
     def execute(self, userdata):
        rospy.loginfo('Executing state FOLLOW_TRAJECTORY')
        print ("Mission Stage coming in: %s " %self.flightStatus.mission_stage)
        rospy.sleep(self.flightStatus.sleepTime)
        if self.flightStatus.PositionErrorDiverge():
            print ("Either pilot wants control back or vehicle is unstable - goto MANUAL")                 
            self.flightStatus.mission_stage = 'INBOUND'
            return 'Aborted_Diverge' #->Manual!
        if not self.flightStatus.IsBatteryOK() or self.flightStatus.listener.MissionGoSwitch == False:
            print ("Either pilot wants vehicle to come home or there is no Batt - goto LAND")#Later should be mapped to GOHOME state
            self.flightStatus.mission_stage  = 'INBOUND'
            return 'Aborted_NoBatt' #->Vehicle should LAND
        if self.flightStatus.IsTimeExceeded() : #Mission Duration Reached
            print ("Mission Duration Exceeded - Finish") 
            self.flightStatus.mission_stage  = 'INBOUND'
            return 'Aborted_NoBatt'
        if self.flightStatus.ErrorConverge('z'):
            print ("Seems like I have arrived at destination --->>> goto HOVER") 
            return 'Arrived'
        print("Following Trajectory")
        self.flightStatus.mission_stage  = 'OUTBOUND'
##        print ("Mission Stage coming out: %s " %userdata.TrajFol_mission_stage_out)
        return 'Maintain'
            

# define state InitContoller
class CONTROLLER_INIT(smach.State):
    def __init__(self,flightStatus,controlManagement , str_ParentStateName):
        smach.State.__init__(self, outcomes=['Success','Failure'])
        self.flightStatus        = flightStatus
        self.controlManagement   = controlManagement
        self.str_ParentStateName = str_ParentStateName #Used to indicate which controller should be turned ON
        self.dict = {'True': 'ON', 'False':'OFF'};
        #self.stateDictionary = {'IDLE': 1, 'HOVER':2,'LAND':3,'TAKEOFF':4};

    def execute(self,userdata):
        rospy.loginfo('Executing state ControllerInit in %s' , self.str_ParentStateName )
        rospy.sleep(self.flightStatus.sleepTime)
        # Create a service client
        Service_in = ControllerRequest()                 
        # Designated whether there are controller gains to be adjusted (Default)
        Service_in.input_gain_flag = True
        # Controller PID gains (regarded only if flag is TRUE, Default)
        Service_in.gains = [1.0,2.0,3.0] 
        # Default  - Controller should turn ON
        Service_in.ON_OFF = True
                
        if self.str_ParentStateName is 'IDLE' or self.str_ParentStateName is 'HOVER': #Either HOVER or IDLE
            if self.str_ParentStateName is 'IDLE':
                                            #<<<---Initializing Controller in IDLE sub state machine--->>>
                #Come in from either LAND or AUTONOMOUS_INIT
                if self.flightStatus.listener.AutoPilotSwitch == False or self.flightStatus.listener.MissionGoSwitch == False or not self.flightStatus.IsBatteryOK():
                    print ('All Controllers should be turned off...') 
                    # Designated that the controller should turn OFF
                    Service_in.ON_OFF = False
                    print ('All Controllers turned off - we are DONE') 
                    #print ("AutoPilot if OFF or MissionGo is OFF  --->>> goto MANUAL") 
                else:
                    print("Getting ready to start mission...")
                    print ("Creating a StampedPose to be used as a constant ref signal for the controller")
                    Service_in.trajectory.append(self.flightStatus.getCurrentPoseStamped()) 
                                             #<<<---Initializing Controller in HOVER sub state machine--->>>
            elif self.str_ParentStateName is 'HOVER':
                print("Starting contoller for HOVER")
                print ("Creating a StampedPose to be used as a constant ref signal for the controller")
                Service_in.trajectory.append(self.flightStatus.getCurrentPoseStamped()) 
        else:                                #<<<---Initializing Controller in For TAKEOFF/LAND sub state machine--->>>
            #Determine target altitude depending on parent state:
            if self.str_ParentStateName is 'TAKEOFF':
                z_final = self.flightStatus.safeAltitude + 0.1
                print 'Generating trajectory for TAKEOFF'
            elif self.str_ParentStateName is 'LAND':
                z_final = self.flightStatus.groundLevel
                print 'Generating trajectory for LAND'
            # Call a function that generates a trajectory for the controller to follow - - >>>> SHOULD BE A SERVICE PROVIDOR
            v_max = 2 #[meters\second] Should be a function of the time horizon and the change in altitude  i.e : v_max = abs(z_final-z_init)/horizon
            delay = 0 #[seconds]
            tolerance = 0.01 #[meters]
            Service_in.trajectory = getTrajectoryAltitudeProfile(self.flightStatus.getCurrentPoseStamped(),
                                                             z_final,
                                                             v_max,
                                                             tolerance)

        # Send service request
        if self.controlManagement.ControllerClient(Service_in):
            print("Controller SUCCEEDED to turn " + self.dict[str(Service_in.ON_OFF)] )
            return 'Success'
        else:
            print("Controller FAILED to turn " + self.dict[str(Service_in.ON_OFF)])
            return 'Failure'

