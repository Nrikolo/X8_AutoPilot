#!/usr/bin/env python

import roslib; roslib.load_manifest('beginner_tutorials')
from beginner_tutorials.srv import *
import rospy
import random


def handle_Controller(req):
    #Display msgs indicating what the service request holds
    print "\n\nInside handle_Controller, returning the service call"
    print ("Received ON_OFF %s" % (req.ON_OFF))
    #print ("Received Trajectory %s" % (req.trajectory))
    print ("Received gain indicator %s" % (req.input_gain_flag))
    print ("Received gain indicator %s" % (req.input_gain_flag))    
    print ("Received Gains : \n Proportional Gain is : %5.3f \n Derivative Gain is : %5.3f \n Integral Gain is : %5.3f" %(req.gains[0], req.gains[1],req.gains[2]))
    
    if req.ON_OFF:
        print("Contorller should be turned ON")
        flag = random.uniform(0,1)>0.1
    else:
        print("Contorller should be turned OFF")
        flag = True
    
    return ControllerResponse(flag)

def ControllerServer():
    rospy.init_node('ControllerServer')
    s = rospy.Service('ManageController', Controller , handle_Controller)
    print "Ready to call controllers, at your service..."
    rospy.spin()

if __name__ == "__main__":
    ControllerServer()
    