#!/usr/bin/env python


import roslib; roslib.load_manifest('fsm')
import rospy
import numpy
import string
#from message_filters import Subscriber, Cache
from std_msgs.msg import  Bool
from smach_msgs.msg import SmachContainerStatus


class FsmMenifoldIndicatorClass():
    def __init__(self):
        rospy.init_node('menifold_indicator')
        print("\nInitializing menifold indicator object!")
        self.topic_in   = 'topic_in'
        self.topic_out  = 'topic_out'
        self.subscriber = rospy.Subscriber(self.topic_in,
                                           SmachContainerStatus,
                                           self.callback)        
        self.publisher  = rospy.Publisher(self.topic_out, Bool) 
        
    def callback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + " I heard  %s  ",  data)
        out = Bool(True)
        #Extract the string            
        str = data.path
##        print str
##        print data.active_states[0]
        #find whether there is MANUAL or AUTONOMOUS parse in the string        
        if str != '/SM_TOP/AUTONOMOUS' :
            return 
        if data.active_states[0]=='None':
##            print "->>>>>>>>>>>MANUAL"
            out.data = False
        else:
            pass
##            print "->>>>>>>>>>>AUTONOMOUS"
            
        
        #publish that bool
        self.publisher.publish(out)     
        

if __name__ == '__main__':
    try:
        converter = FsmMenifoldIndicatorClass()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass