#!/usr/bin/env python

""" This file defines the Listener class and its methods. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
from std_msgs.msg import Bool

import math
from numpy  import *
import threading

import smach
import smach_ros
import Listener
#from smach import State, StateMachine, Sequence
#from smach_ros import ServiceState, SimpleActionState


#Class WaitForMsgState inharits from smach.State class
class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.
    
    It is meant to be extended with a case specific class that initializes this one appropriately 
    and contains the msg_cb (or overloads execute if really needed).
    
    Its waitForMsg method implements the core functionality: waiting for the message, returning 
    the message itself or None on timeout.
    
    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned 
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will 
    be called with the message and the userdata, so that a self defined method can convert message data to 
    smach userdata.
    Those userdata fields have to be passed via 'output_keys'.
    
    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.
    
    If thats still not enough, execute() might be overloaded.
    
    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to 60.
    output_keys: Userdata keys that the message callback needs to write to. 
    """
    # might need to define buffer_size in subscriber!!

    def __init__(self, topic, msg_type, listener, msg_cb=None, output_keys=[], latch=False, timeout=60, queue_size=10):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],  output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size)
        self.listener = listener

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        '''Await and return the message or None on timeout.'''
        print 'Waiting for message...'
        timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg != None:
                print 'Got message.'
                message = self.msg
                
                if not self.latch:
                    self.msg = None
                
                self.mutex.release()
                return message
            self.mutex.release()
            rospy.sleep(.1)
        
        print 'Timeout!'
        return None

    def execute(self, ud):
        
        batteryok = self.listener.IsBatteryOK()
        throttle = self.listener.ThrottleLevel()
        '''Default simplest execute(), see class description.'''
        msg = self.waitForMsg()
        if msg != None:
            # call callback if there is one
            if self.msg_cb != None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result != None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'



