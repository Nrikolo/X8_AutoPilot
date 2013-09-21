#Listener class object
#!/usr/bin/env python#

PKG = 'rospy_tutorials' # this package name
import roslib; roslib.load_manifest(PKG)
import rospy
import numpy
import time
import message_filters
#from message_filters import Subscriber, Cache
from std_msgs.msg import Float64, Bool, Int16
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

from rospy.numpy_msg import numpy_msg 

class ListenerClass():
    def __init__(self):
        print("Initializing Listener Object!")
        #Define topics to which the listener should subscribe (should be imported from a parameter server later)
        self.StartedListening = rospy.Time.now()
        batteryVoltage_topic  = 'battery'
        poseStamped_topic     = 'poseStamped'
        AutoPilotSwitch_topic = 'AUTOPILOT'
        ctrlThrottle_topic    = 'ctrl_throttle'
        MissionGoSwitch_topic = 'MISSIONGO'
        #Create subscribers to those topics (msg_type should also be loaded from server)
        print("Starting to Listen!")
        self.subscriber_batteryVoltage = rospy.Subscriber(batteryVoltage_topic,
                                                          Float64,
                                                          self.battery_callback)        
        self.subscriber_pose           = rospy.Subscriber(poseStamped_topic,
                                                          PoseStamped,
                                                          self.poseStamped_callback)       
        #self.pose_cache = message_filters.Cache(self.subscriber_pose, 100)
        #self.pose_cache.registerCallback(self.pose_callback)
        
        self.subscriber_AP_switch      = rospy.Subscriber(AutoPilotSwitch_topic,
                                                          Bool,
                                                          self.AutoPilotSwitch_callback)     
        self.subscriber_ctrl_throttle  = rospy.Subscriber(ctrlThrottle_topic,
                                                          Int16,
                                                          self.ctrlThrottle_callback)     
        self.subscriber_MissionGo      = rospy.Subscriber(MissionGoSwitch_topic,
                                                          Bool,
                                                          self.MissionGo_callback)                                                             
        #Initialize the data loggers for the listener 
        self.batteryVoltage   = 15
        self.poseStampedQueue = [] 
        self.ctrlThrottle     = 0
        self.AutoPilotSwitch  = True
        self.MissionGoSwitch  = True
        

    def battery_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data.data)
        self.batteryVoltage  = data.data
        
    def poseStamped_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        self.poseStampedQueue.append(data) #Append the latest msg to the array, column-wise
        if len(self.poseStampedQueue)>100:
            self.poseStampedQueue.pop(0) #Erase the oldest msg  from the list
            
    def AutoPilotSwitch_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        self.AutoPilotSwitch  = data.data

    def ctrlThrottle_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data.data)
        self.ctrlThrottle  = data.data

    def MissionGo_callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        self.MissionGoSwitch  = data.data
        
  