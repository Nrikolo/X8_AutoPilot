#!/usr/bin/env python

import roslib; roslib.load_manifest('beginner_tutorials')
from beginner_tutorials.srv import *
from nav_msgs.srv import GetPlan 
from nav_msgs.msg import Path
import rospy
import random


def getTrajectory(currentPose, finalPose, tolerance): 
    #Function should be formulated as a motion planner wrapped as a service
    """
    :param currentPose: A ros/geometry_msgs/Pose message
    :param finalPose: A ros/geometry_msgs/Pose message
    :param tolerance : A float indicating maximal tolerance between sigamoid an actual height
    :return: An array of ros/geometry_msgs/PoseStamped messages representing a trajectory
    
    Generates a time stamped trajectory between initial and final pose
    """  
    print ("Genereting trajectory from initPose =  {}  -->> finalPose = {} [m]".format( currentPose, finalPose))    
    #Ref signal frequency
    frequency         = 100 #[Hz]
    #Use current position [x,y] constant and current z as lower asymptote of sigamoid
    x       = currentPose.pose.position.x
    y       = currentPose.pose.position.y
    z_init  = currentPose.pose.position.z
    quat    = Quaternion(0.0, 0.0, 0.0, 1.0)
    z_final = fianlPose.pose.position.z
    delay   = math.log(abs(z_init-z_final)/tolerance-1)/v_max
    #Loop to populate trajectory with StampedPoses instances
    trajectory = [] # z = z_init + (z_final - z_init)./(1+exp(-v_max*(t-delay-tNOW))));    
    TrajectoryStartTime = rospy.Time()
    i=0
    while True: #no more than 10 sec
        StampedPose                 = PoseStamped() #Construct a StampedPose MSG        
        StampedPose.header.frame_id = "/Body" #Frame of ref that the trajectory is formualted in
        t                           = TrajectoryStartTime + rospy.Duration(float(i)/frequency)        #create a time instance
        z                           = z_init + (z_final - z_init)/(1+math.exp(-v_max*(t.to_sec()-delay)));    #compute its height
        #print("Test traj : At time {} z value is {}".format(t.to_sec(),z))
        #Populate the StampedPose object
        StampedPose.header.stamp    = t
        StampedPose.pose            = Pose(Point(x, y, z),quat) #The pose
        # Append the time staped pose to the trajectory to follow
        trajectory.append(StampedPose)
        i += 1
        if abs(z-z_final) < tolerance:
            #print ("trajectory length is {}".format(i))            
            break
        elif i>10*frequency:
            print ("Trajectory is more than 10 seconds long, truncated now! ")
            break
    #print ("\nTrajectory start is: {}".format(trajectory[0]))
    #print ("\nTrajectory end is: {}".format(trajectory[-1]))
    return trajectory

def handle_TrajectoryPlanner(req):
    #Display msgs indicating what the service request holds
    print "\n\nInside handle_TrajectoryPlanner, returning the service call"
    print ("Received Start Position " % (req.initPose))
    print ("Received Goal Position " % (req.goalPose))
    print ("Received tolerance " % (req.tolerance))
     
    Trajectory = getTrajectoryAltitudeProfile(req.initPose.pose, req.goalPose.pose, req.tolerance) 

    return TrajectoryPlannerResponse(Trajectory)

def TrajectoryPlannerServer():
    rospy.init_node('TrajectoryPlannerServer')
    s = rospy.Service('TrajectoryPlanner', GetPlan , handle_TrajectoryPlanner)
    print "Ready to call trajectory planner, at your service..."
    rospy.spin()

if __name__ == "__main__":
    TrajectoryPlannerServer()
    