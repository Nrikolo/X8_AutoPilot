 #!/usr/bin/env python#
# A set of utility functions for converting msgs to numpy arrays, computing distance metrics 

import rospy
import ros
import numpy
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

class switch(object):
    """
    python switch statement adapted from : http://code.activestate.com/recipes/410692-readable-switch-construction-without-lambdas-or-di/
    """
    def __init__(self, value):
        self.value = value
        self.fall = False

    def __iter__(self):
        """Return the match method once, then stop"""
        yield self.match
        raise StopIteration
    
    def match(self, *args):
        """Indicate whether or not to enter a case suite"""
        if self.fall or not args:
            return True
        elif self.value in args: 
            self.fall = True
            return True
        else:
            return False

def PoseMsg2NumpyArray(pose_msg):
    """
    :param: pose_msg: A ros/geometry_msgs/Pose type msg
    :return: A numpy array size [7,1] holds [x,y,z,q1,q2,q3,q0] where q0 is the quaternion scalar
    
    Converts ros/geometry_msgs/Pose to a numpy array
    """
    #print "PoseMsg2NumpyArray : ", pose_msg
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
    :param pose_msg: A ros/geometry_msgs/Pose type msg
    :return: A numpy array size [3,1] holds [x,y,z]
    
    Converts ros/geometry_msgs/Pose to a numpy array
    """
    #print "PoseMsg2NumpyArrayPosition: " , pose_msg
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

def EuclideanPlanarDistance( array_1, array_2,dim):
    """
    :param array_1 : A numpy array
    :param array_2 : A numpy array
    :param dim : An integer  numpy array
    :return: A float measuring the Euclidean distance two arrays
    
    Computed the Euclidean metric of the first dim  dimesions between two numpy arrays"""
##    print "\narray_1:", array_1
##    print "\narray_2:", array_2
    diff = array_1[0:dim]-array_2[0:dim]    
##    print "\ndifference between arrays", diff
    return numpy.linalg.norm(diff,2)


def ManhattenPlanarDistance( array_1, array_2, dim=3):
    """
    :param array_1 : A numpy array
    :param array_2 : A numpy array
    :param dim : An integer  numpy array
    :return: A float measuring the Manhatten distance two arrays
    
    Computed the Manhatten metric of the first dim dimesions between two numpy arrays"""

    diff = array_1[0:dim]-array_2[0:dim]    
    return numpy.linalg.norm(diff,1)


def getTrajectoryAltitudeProfile(currentPoseStamped, TargetPose, tolerance): 
    #Function should be formulated as a motion planner wrapped as a service
    """
    :param currentPoseStamped: A ros/geometry_msgs/PoseStamped message
    :param TargetPose : A ros/geometry_msgs/Pose message indicating target pose
    :param tolerance : A float indicating maximal tolerance between sigamoid an actual height
    :return: An array of ros/geometry_msgs/PoseStamped messages
    
    Generates an altitude proflie for TAKEOFF/LAND based on a sigamoid
    """  
    
    #Ref signal frequency
    frequency         = 100 #[Hz]
    #Use current position [x,y] constant and current z as lower asymptote of sigamoid
    x       = currentPoseStamped.pose.position.x
    y       = currentPoseStamped.pose.position.y
    z_init  = currentPoseStamped.pose.position.z
    z_final = TargetPose.position.z
    quat    = Quaternion(0.0, 0.0, 0.0, 1.0)
    v_max   = abs(z_final-z_init)/2.0 #[meters\second] Should be a function of the time horizon and the change in altitude  i.e : v_max = abs(z_final-z_init)/horizon    
    print (abs(z_init-z_final)/tolerance) - 1    
    delay   = math.log((abs(z_init-z_final)/tolerance) - 1)/v_max
    #Loop to populate trajectory with StampedPoses instances
    trajectory = [] # z = z_init + (z_final - z_init)./(1+exp(-v_max*(t-delay-tNOW))));    
    print ("\nGenereting an altitude profile from z =  {} [m] -->> z = {} [m]".format(z_init , z_final))
    i=0
    while True: #no more than 10 sec
        StampedPose                 = PoseStamped() #Construct a StampedPose MSG        
        StampedPose.header.frame_id = "/world" #Frame of ref that the trajectory is formualted in
        t                           = rospy.Duration(float(i)/frequency)        #create a time instance
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

    print ("\nTrajectory start is: {}".format(trajectory[0].pose.position))
    print ("\nTrajectory end is: {}".format(trajectory[-1].pose.position))
    return trajectory

def getTrajectory(currentPose, TargetPose , tolerance = 0.01): 
    #Function should be formulated as a motion planner wrapped as a service
    """
    :param currentPoseStamped: A ros/geometry_msgs/PoseStamped message
    :param TargetPose : A ros/geometry_msgs/Pose message indicating target pose
    :param tolerance : A float indicating maximal tolerance between sigamoid an actual height
    :return: An array of ros/geometry_msgs/PoseStamped messages
    
    Generates a trajectory the vehicle to follow (sigamoid based)
    """  
    trajectory = []
      
    distance_to_target = Distance('Euclidean',
                                   PoseMsg2NumpyArrayPosition( currentPose ),
                                   PoseMsg2NumpyArrayPosition( TargetPose ),
                                   3)    
##    print "Euclidean Distance to Target: " , distance_to_target 
    if distance_to_target < tolerance :
        print "Basically at target already"
        StampedPose                 = PoseStamped() #Construct a StampedPose MSG        
        StampedPose.header.frame_id = "/world" #Frame of ref that the trajectory is formualted in
        StampedPose.header.stamp    = rospy.Time.now()
        StampedPose.pose            = currentPose #The pose
        trajectory.append(StampedPose)
        return trajectory

    #Ref signal frequency
    frequency         = 100 #[Hz]
    #Use current position [x,y] constant and current z as lower asymptote of sigamoid
    x_init  = currentPose.position.x
    y_init  = currentPose.position.y
    z_init  = currentPose.position.z
    x_final = TargetPose.position.x
    y_final = TargetPose.position.y
    z_final = TargetPose.position.z
     
    quat    = Quaternion(0.0, 0.0, 0.0, 1.0)
    delta   = numpy.amax([abs(z_final-z_init),abs(z_final-z_init),abs(z_final-z_init)])
    if delta<tolerance:
        v_max   = 1.0 #[meters\second] Should be a function of the time horizon and the change in altitude  i.e : v_max = abs(z_final-z_init)/horizon    
        delay   = 0.0
    else:
        v_max   = delta/1.0 #[meters\second] Should be a function of the time horizon and the change in altitude  i.e : v_max = abs(z_final-z_init)/horizon    
        delay   = math.log(delta/tolerance - 1)/v_max
        
    #Loop to populate trajectory with StampedPoses instances
    #print ("\nGenereting a trajectory from z =  {} [m] -->> z = {} [m]".format(z_init , z_final))
    i=0
    while True: 
        StampedPose                 = PoseStamped() #Construct a StampedPose MSG        
        StampedPose.header.frame_id = "/world" #Frame of ref that the trajectory is formualted in
        t                           = rospy.Duration(float(i)/frequency)        #create a time instance
        den                         = (1+math.exp(-v_max*(t.to_sec()-delay)))  
        x                           = x_init + (x_final - x_init)/den    #compute its height
        y                           = y_init + (y_final - y_init)/den
        z                           = z_init + (z_final - z_init)/den
##        print("Test traj : At time {} x value is {}".format(t.to_sec(),x))
        #Populate the StampedPose object
        StampedPose.header.stamp    = t
        StampedPose.pose            = Pose(Point(x, y, z),quat) #The pose
        # Append the time staped pose to the trajectory to follow
        trajectory.append(StampedPose)
        i += 1
        bool = numpy.array([abs(x-x_final),abs(y-y_final),abs(z-z_final)])<tolerance
        #print bool
        if bool.all():
##            print ("Trajectory length is {}".format(i))            
            break
##    print "\nTrajectory generated."    
    print ("\nTrajectory start is:\n {}".format(trajectory[0].pose.position))
    print ("\nTrajectory end is:\n {}".format(trajectory[-1].pose.position))
    return trajectory
