#!/usr/bin/env python
import roslib; roslib.load_manifest('fsm')

import sys

import rospy
#from beginner_tutorials.srv import *
from quadrotor_input.srv import *

class ControllerManagementClass():
    def __init__(self):
        while True:
            try:
                rospy.wait_for_service('controller/command', 1.0)
                break
            except rospy.ROSException , error:
                print error.message        
            except :
                print "break"
                break
        
        self.handle_service = rospy.ServiceProxy('controller/command', CommandController)        
        
    #def ControllerClient(self, ON_OFF, trajectory, input_gain_flag, gains):
    def ControllerClient(self, service_in):
        try:
            resp = self.handle_service(service_in.running, service_in.path, service_in.gains)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

