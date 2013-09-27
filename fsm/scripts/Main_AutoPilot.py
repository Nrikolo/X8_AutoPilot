#!/usr/bin/env python
#General Imports
import roslib; #roslib.load_manifest('smach_tutorials')
roslib.load_manifest('fsm')
import rospy
import time
import random
#Importing SMACH package 
import smach
import smach_ros
#For dealing with msgs
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Pose, Point, Quaternion

#Importing the Flight Status/Error Indicator class definition
from FlightStatusClass import FlightStatusClass
#Importing the Controller Management Class definition
from ControllerManagementClass import ControllerManagementClass
#Importing the States definitions 
from fsm_states import *

def main():
    rospy.init_node('TOP_FSM')

    # Create a SMACH state machine
    # The FSM has one outcome of "Done" when mission exceedes alloted time or otherwise defined in the transitions
    sm_top = smach.StateMachine(outcomes=['Done'])

    #Safety Parameters (there is no const in python, caferful when accesing these from within the sub_fsms)
    safeAltitude        = 1.0
    BattMinimalVoltage  = 13.8
    groundLevel         = 0.2
    throttleThreshold   = 0.2
    missionMaxTime      = rospy.Duration(300).to_sec() #300 sec = 5 min

    #Define Static Parameters 
    sm_top.userdata.Link = 1 #Can be used to transfer data between states, already remaped and stitched
    home                 = Point(0.0, 0.0, 1.0)#Default Home [x,y,z] position, msg type Point of geometry_msgs 
    tolerance            = 0.02 #[m] for error signal convergence
    queue_size           = 50 #Size of history queue of pose msgs saved for convergence computation
    dictionary           = {'x' : 0, 
                            'y' : 1,
                            'z' : 2,
                            'qx': 3,
                            'qy': 4,
                            'qz': 5,
                            'q0': 6}
    
    #Define Debugging Aid
    fsm_refresh_rate     = 1.0 #A time interval in seconds [float] to wait when entering each state - used as rospy.sleep(fsm_refresh_rate)
    
    pub_AutonomousMode = rospy.Publisher('AutonomousMode',Bool)
    
    #Contruct a FlightStatusIndicator as a member of sm_top
    sm_top.FlightStatus = FlightStatusClass(dictionary,
                                            queue_size,
                                            BattMinimalVoltage,
                                            safeAltitude,
                                            groundLevel,
                                            throttleThreshold,
                                            missionMaxTime,
                                            home,
                                            fsm_refresh_rate,
                                            tolerance)
    #Contruct a ControlManager as a member of sm_top
    sm_top.ControlManager = ControllerManagementClass()
        
    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('MANUAL', MANUAL(sm_top.FlightStatus,pub_AutonomousMode), 
                                transitions={'TOAUTONOMOUS':'AUTONOMOUS', 'Finish':'Done'},
                                remapping = {'manual_Home':'Link'})        
#----------------------------------------------------------------------------------------
        # Create an Autonomous (manifold) FSM within the top container
        sm_autonomous = smach.StateMachine(outcomes=['Success','Failure'],
                                           input_keys=['Auto_Home'])
        # Open the autonomous container
        with sm_autonomous:

        # Create and add the AutonomousInit SMACH state 
            smach.StateMachine.add('AUTONOMOUS_INIT', AUTONOMOUS_INIT(sm_top.FlightStatus,pub_AutonomousMode), 
                                    transitions={'ToIdle':'IDLE', 'ToHover':'HOVER', 'ToLand':'LAND','Failure':'Failure'},
                                    remapping = {'AutoInit_Home':'Auto_Home'})      
#----------------------------------------------------------------------------------------
    # Create the IDLE SMACH state machine
            sm_idle = smach.StateMachine(outcomes=['ToTakeoff','ToManual'],input_keys=['I_Home'])      
            with sm_idle:
                # Create and add the HOVER_INIT SMACH state
                smach.StateMachine.add('IDLE_INIT',CONTROLLER_INIT(sm_top.FlightStatus,sm_top.ControlManager,'IDLE'),transitions={'Success':'IDLE_MONITOR',
                                                                                                       'Failure':'ToManual'},
                                                                                            remapping={'idle_Home':'I_Home'})
                # Create and add the HOVER_MONITOR SMACH state
                smach.StateMachine.add('IDLE_MONITOR',IDLE(sm_top.FlightStatus,pub_AutonomousMode ),transitions={'Finish':'ToManual',
                                                                                        'Start':'ToTakeoff',
                                                                                        'Maintain':'IDLE_MONITOR'},
                                                                            remapping={'idle_Home':'I_Home'})
            #Add a IDLE sub state machine to the autonomous manifold                                                            
            smach.StateMachine.add('IDLE',sm_idle, transitions={'ToTakeoff':'TAKEOFF','ToManual':'Failure'},
                                                     remapping={'I_Home':'Auto_Home'})
#----------------------------------------------------------------------------------------         
            # Create and add the TAKEOFF SMACH Container state machine
            sm_takeoff = smach.StateMachine(outcomes=['ToHover','ToLand','ToManual'],
                                           input_keys=['TO_Home'])        
            with sm_takeoff:
        # Create and add the TAKEOFF_INIT state into the takeoff substate machine
                smach.StateMachine.add('TAKEOFF_INIT',CONTROLLER_INIT(sm_top.FlightStatus,sm_top.ControlManager,'TAKEOFF'),
                                                                        transitions={'Success':'TAKEOFF_MONITOR',
                                                                                     'Failure':'ToManual'},
                                                                        remapping={'takeoff_Home':'TO_Home'})    
        # Create and add the TAKEOFF_MONITOR state into the takeoff substate machine
                smach.StateMachine.add('TAKEOFF_MONITOR',TAKEOFF(sm_top.FlightStatus,pub_AutonomousMode ),transitions={'Success':'ToHover',
                                                                                            'Maintain':'TAKEOFF_MONITOR',
                                                                                            'Aborted_NoBatt':'ToLand',
                                                                                            'Aborted_Diverge':'ToManual'},
                                                                               remapping={'takeoff_Home':'TO_Home'})
        #Add a Takeoff sub state machine to the autonomous manifold
            smach.StateMachine.add('TAKEOFF', sm_takeoff,transitions={'ToHover':'HOVER',
                                                                  'ToLand':'LAND',
                                                                  'ToManual':'Failure'},
                                                            remapping={'TO_Home':'Auto_Home'})
#----------------------------------------------------------------------------------------
    # Create the LAND SMACH state machine
            sm_land = smach.StateMachine(outcomes=['Success','Failure'],input_keys=['L_Home'])      
            with sm_land:
                # Create and add the LAND_INIT SMACH state into the LAND substate machine
                smach.StateMachine.add('LAND_INIT',CONTROLLER_INIT(sm_top.FlightStatus,sm_top.ControlManager,'LAND'),
                                                                    transitions={'Success':'LAND_MONITOR',
                                                                                'Failure':'Failure'},
                                                                    remapping={'land_Home':'L_Home'})
                                                                        
                # Create and add the LAND_MONITOR SMACH state into the LAND substate machine
                smach.StateMachine.add('LAND_MONITOR',LAND(sm_top.FlightStatus,pub_AutonomousMode ),transitions={'Success':'Success',
                                                                                         'Maintain':'LAND_MONITOR',
                                                                                         'Failure':'Failure'},
                                                            remapping={'land_Home':'L_Home'})
            #Add a Land sub state machine to the autonomous manifold                                                            
            smach.StateMachine.add('LAND',sm_land,transitions={'Success':'IDLE','Failure':'Failure'},
                                                            remapping={'L_Home':'Auto_Home'})
#----------------------------------------------------------------------------------------
    # Create the HOVER SMACH state machine
            sm_hover = smach.StateMachine(outcomes=['ToLand','ToManual'],input_keys=['H_Home'])      
            with sm_hover:
                # Create and add the HOVER_INIT SMACH state into the HOVER sub state machine
                smach.StateMachine.add('HOVER_INIT',CONTROLLER_INIT(sm_top.FlightStatus,sm_top.ControlManager,'HOVER'),
                                                                    transitions={'Success':'HOVER_MONITOR',
                                                                                'Failure':'ToManual'},
                                                                    remapping={'hover_Home':'H_Home'})
                # Create and add the HOVER_MONITOR SMACH state into the HOVER sub state machine
                smach.StateMachine.add('HOVER_MONITOR',HOVER(sm_top.FlightStatus,pub_AutonomousMode ),transitions={'MissionDone':'ToLand',
                                                                                         'Maintain':'HOVER_MONITOR',
                                                                                        'Aborted_NoBatt':'ToLand',
                                                                                        'Aborted_Diverge':'ToManual'},
                                                                            remapping={'hover_Home':'H_Home'})
            #Add a HOVER sub state machine to the autonomous manifold                                                            
            smach.StateMachine.add('HOVER',sm_hover, transitions={'ToLand':'LAND','ToManual':'Failure'},
                                                     remapping={'H_Home':'Auto_Home'})
#----------------------------------------------------------------------------------------                                   
    # Create the GO_HOME state machine
            sm_goHome = smach.StateMachine(outcomes=['ToHover','ToManual'],input_keys=['Gohome_Home'])      
            with sm_goHome:
                # Create and add the GO_HOME_INIT SMACH state into the GO_HOME sub state machine
                smach.StateMachine.add('GO_HOME_INIT',CONTROLLER_INIT(sm_top.FlightStatus,sm_top.ControlManager,'GO_HOME'),
                                                                    transitions={'Success':'GO_HOME_MONITOR',
                                                                                'Failure':'ToManual'},
                                                                    remapping={'g_Home':'Gohome_Home'})
                # Create and add the HOVER_MONITOR SMACH state into the HOVER sub state machine
                smach.StateMachine.add('GO_HOME_MONITOR',FOLLOW_TRAJECTORY(sm_top.FlightStatus,pub_AutonomousMode ),transitions={'Arrived':'ToHover',
                                                                                         'Maintain':'GO_HOME_MONITOR',
                                                                                        'Aborted_NoBatt':'ToManual',
                                                                                        'Aborted_Diverge':'ToManual'},
                                                                            remapping={'g_Home':'Gohome_Home'})
            #Add a HOVER sub state machine to the autonomous manifold                                                            
            smach.StateMachine.add('GO_HOME',sm_goHome, transitions={'ToHover':'HOVER','ToManual':'Failure'},
                                                     remapping={'Gohome_Home':'Auto_Home'})
#----------------------------------------------------------------------------------------                                   
        #Add the Autonomous (manifold) state to the top container state machine
        smach.StateMachine.add('AUTONOMOUS', sm_autonomous,transitions={'Success':'Done',
                                                                        'Failure':'MANUAL'},
                                                            remapping={'Auto_Home':'Link'})
#----------------------------------------------------------------------------------------                                   

    # Create and start the introspection server for visualization of the FSm
    sis = smach_ros.IntrospectionServer('FSM', sm_top, '/SM_TOP')
    sis.start()
    #Wait till come data is accumulated (alternative??)
    rospy.sleep(2.)
    # Execute SMACH plan
    outcome = sm_top.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
