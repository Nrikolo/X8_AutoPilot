#!/usr/bin/env python
#A node that emulated the topics as if they are generated by the vehicle. 
# Topics are the vehicle pose, battery voltage, the throttle stick RC input 
# and lastly the Autopilot and mission go boolean indicators. 

import roslib; roslib.load_manifest('beginner_tutorials')
import random
import math
import rospy
import sys

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Vector3
from quadrotor_msgs.msg import RadioControl, BatteryStatus, ControllerError


def X8_Emulator():
    #State a ROS NODE    
    rospy.init_node('X8_HardwareEmulator', anonymous=True)    
    #Create Publishers    
    pub_poseStamped = rospy.Publisher('x8/output/pose', PoseStamped) 
    pub_batt        = rospy.Publisher('x8/output/battery', BatteryStatus)
    pub_RadioCtrl   = rospy.Publisher('x8/input/radio_control', RadioControl)
    pub_CtrlError   = rospy.Publisher('controller/error', ControllerError)
        
    r = rospy.Rate(5) # 10hz    
    start_time = rospy.Time.now().to_sec()
    AutoPilotSwitch =       True
    MissionGoSwitch =       True
    poseStamped     = PoseStamped() #Construct a Stamped Pose Msg
    battery         = BatteryStatus() #Construct a Battery Status Msg
    CtrlError       = ControllerError() #Construct a ControllerError Msg
    freq            = 0.002 #Frequancy of sinosoidal wave for throttle stick
    while not rospy.is_shutdown():
        #Generate Signals
        t                           = rospy.Time.now().to_sec()-start_time
        poseStamped.pose            = Pose(Point(2*math.cos(2*math.pi*freq*t), math.sin(2*math.pi*freq*t), math.fabs(1.0+2*math.sin(2*math.pi*freq*t))), Quaternion(0.000, 0.000, 0.000, 1.00))
        poseStamped.header.frame_id = "/world" #Frame of ref that the trajectory is formualted in
        poseStamped.header.stamp    = rospy.Time.now()
        
        battery.voltage             = 15-0.01*t
        battery.current             = 1.2
        battery.header.stamp        = rospy.Time.now()
        RadioSignal                 = RadioControl(math.cos(2*math.pi*freq*t),#roll
                                                   math.cos(2*math.pi*freq*t),#pitch
                                                   math.cos(2*math.pi*freq*t),#yaw
                                                   abs(math.sin(2*math.pi*freq*t/10)),#throttle
                                                   1,           #flap 
                                                   1)           #Not relevant
        CtrlError.header.frame_id   = "/world"
        CtrlError.header.stamp      = rospy.Time.now()
        CtrlError.error             = Vector3(math.cos(2*math.pi*freq*t),
                                              math.cos(2*math.pi*freq*t),
                                              math.cos(2*math.pi*freq*t))
        CtrlError.error_d           = Vector3(2*math.pi*freq*math.sin(2*math.pi*freq*t),
                                              2*math.pi*freq*math.sin(2*math.pi*freq*t),
                                              2*math.pi*freq*math.sin(2*math.pi*freq*t))
        
        
        #Log Signals        
        rospy.loginfo("AutoPilotSwitch: %s", RadioSignal.flap > 0.5)
        rospy.loginfo("MissionGoSwitch: %s", RadioSignal.gear > 0.5)
        rospy.loginfo("Throttle Command: %s", RadioSignal.throttle)        
        rospy.loginfo("Battery Voltage: %s" , battery.voltage)
        #rospy.loginfo("Controller Error  %s"  , CtrlError)        
        rospy.loginfo("Vehicle Pose:\n %s"  , poseStamped.pose)
        
        print("---------------------------------------------")
        #Publish Signals
        pub_CtrlError.publish(CtrlError)        
        pub_RadioCtrl.publish(RadioSignal)
        pub_poseStamped.publish(poseStamped)
        pub_batt.publish(battery)
        #Sleep (monitor the publishing rate)
        r.sleep()
    
if __name__ == '__main__':
    try:
        X8_Emulator()
    except rospy.ROSInterruptException: pass

