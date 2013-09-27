 #!/usr/bin/env python#
# A set of utility functions for converting msgs to numpy arrays, computing distance metrics 

import rospy
import ros
import numpy
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped


def PoseMsg2NumpyArray(pose_msg):
    """
    :param: pose_msg: A ros/geometry_msgs/Pose type msg
    :return: A numpy array size [7,1] holds [x,y,z,q1,q2,q3,q0] where q0 is the quaternion scalar
    
    Converts ros/geometry_msgs/Pose to a numpy array
    """
    v =  numpy.array([pose_msg.position.x,
                    pose_msg.position.y,
                    pose_msg.position.z,
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w])
    return v

def PoseMsg2NumpyArrayPosition(pose_msg):
    """
    :param pose_msg: A ros/geometry_msgs/Position type msg
    :return: A numpy array size [3,1] holds [x,y,z]
    
    Converts ros/geometry_msgs/Pose.position to a numpy array
    """
    
    v = PoseMsg2NumpyArray(pose_msg)
    v = v[0:3]
    return v

def PointMsg2NumpyArrayPosition(point_msg):
    """
    :param point_msg: A ros/geometry_msgs/Point type msg
    :return: A numpy array size [3,1] holds [x,y,z]
    
    Converts ros/geometry_msgs/Pose.position to a numpy array
    """

    v =  numpy.array([point_msg.x,
                     point_msg.y,
                     point_msg.z])  
    return v

def Distance(str_metric, array_1, array_2, dim):
    """
    :param str_metric: A string naming the type of metric to be used
    :param array_1 : A numpy array
    :param array_2 : A numpy array
    :param dim : An integer  numpy array
    :return: A float measuring the distance in dimension dim between the two arrays according to the metric 
    
    Computed the distance metric specified in str_metric between two numpy arrays
    """


    if numpy.size(array_1) != numpy.size(array_2):
        print ("Arrays are not of equal size")
        return numpy.inf
    else:            
        if str_metric is 'Manhatten':
            return ManhattenPlanarDistance(array_1, array_2,dim)
        else:
            return EuclideanPlanarDistance(array_1, array_2,dim)

def EuclideanPlanarDistance( array_1, array_2, dim):
    """
    :param array_1 : A numpy array
    :param array_2 : A numpy array
    :param dim : An integer  numpy array
    :return: A float measuring the Euclidean distance two arrays
    
    Computed the Euclidean metric of the first dim  dimesions between two numpy arrays"""
    
    diff = array_1[0:dim-1]-array_2[0:dim-1]    
    return numpy.linalg.norm(diff,2)


def ManhattenPlanarDistance( array_1, array_2, dim):
    """
    :param array_1 : A numpy array
    :param array_2 : A numpy array
    :param dim : An integer  numpy array
    :return: A float measuring the Manhatten distance two arrays
    
    Computed the Manhatten metric of the first dim dimesions between two numpy arrays"""

    diff = array_1[0:dim-1]-array_2[0:dim-1]    
    return numpy.linalg.norm(diff,1)


def getTrajectoryAltitudeProfile(currentPoseStamped, z_final, v_max, tolerance): 
    #Function should be formulated as a motion planner wrapped as a service
    """
    :param currentPoseStamped: A ros/geometry_msgs/PoseStamped message
    :param z_final : A float indicating target altitude
    :param v_max : A float indicating maximal rate
    :param tolerance : A float indicating maximal tolerance between sigamoid an actual height
    :return: An array of ros/geometry_msgs/PoseStamped messages
    
    Generates an altitude proflie for TAKEOFF/LAND based on 
    """  
    
    #Ref signal frequency
    frequency         = 100 #[Hz]
    #Use current position [x,y] constant and current z as lower asymptote of sigamoid
    x      = currentPoseStamped.pose.position.x
    y      = currentPoseStamped.pose.position.y
    z_init = currentPoseStamped.pose.position.z
    quat   = Quaternion(0.0, 0.0, 0.0, 1.0)
    delay  = math.log(abs(z_init-z_final)/tolerance-1)/v_max
    #Loop to populate trajectory with StampedPoses instances
    trajectory = [] # z = z_init + (z_final - z_init)./(1+exp(-v_max*(t-delay-tNOW))));    
    TrajectoryStartTime = rospy.Time()
    print ("Genereting an altitude profile from z =  {} [m] -->> z = {} [m]".format(z_init , z_final))
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