#!/usr/bin/env python
import roslib; roslib.load_manifest('beginner_tutorials')

import sys

import rospy
from beginner_tutorials.srv import *

class ControllerManagementClass():
    def __init__(self):
        rospy.wait_for_service('ManageController')
        self.handle_service = rospy.ServiceProxy('ManageController', Controller)        
        
    #def ControllerClient(self, ON_OFF, trajectory, input_gain_flag, gains):
    def ControllerClient(self, service_in):
        try:
            resp = self.handle_service(service_in.ON_OFF, service_in.trajectory, service_in.input_gain_flag, service_in.gains)
            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
