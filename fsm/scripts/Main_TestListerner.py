#!/usr/bin/env python
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

PKG = 'rospy_tutorials' # this package name
import roslib; roslib.load_manifest(PKG)


import rospy
from std_msgs.msg import *
from ListenerClass import ListenerClass

def main():
    print("In main.py function")
    lis = ListenerClass()
    lis.listen("chatter",Float64)
    
if __name__ == '__main__':
   main()

