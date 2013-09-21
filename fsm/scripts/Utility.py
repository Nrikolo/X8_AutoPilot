 #!/usr/bin/env python#
# A set of utility functions for converting msgs to numpy arrays, computing distance metrics 

import numpy
import rospy
import ros

    

def PoseMsg2NumpyArray(pose_msg):
    """Converts ros/geometry_msgs/Pose to a numpy array"""
    v =  numpy.array([pose_msg.position.x,
                    pose_msg.position.y,
                    pose_msg.position.z,
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w])
    return v

def PoseMsg2NumpyArrayPosition(pose_msg):
    """Converts ros/geometry_msgs/Pose.position to a numpy array"""
    v = PoseMsg2NumpyArray(pose_msg)
    v = v[0:3]
    return v

def PointMsg2NumpyArrayPosition(point_msg):
    """Converts ros/geometry_msgs/Point to a numpy array"""
    v =  numpy.array([point_msg.x,
                     point_msg.y,
                     point_msg.z])  
    return v

def Distance(str_metric, array_1, array_2, dim):
    """Computed the distance metric specified in str_metric between two 3D numpy arrays"""
    if numpy.size(array_1) != numpy.size(array_2):
        print ("Arrays are not of equal size")
        return numpy.inf
    else:            
        if str_metric is 'Manhatten':
            return ManhattenPlanarDistance(array_1, array_2,dim)
        else:
            return EuclideanPlanarDistance(array_1, array_2,dim)

def EuclideanPlanarDistance( array_1, array_2, dim):
    """Computed the Euclidean metric of the first dim  dimesions between two numpy arrays"""
    diff = array_1[0:dim-1]-array_2[0:dim-1]    
    return numpy.linalg.norm(diff,2)


def ManhattenPlanarDistance( array_1, array_2, dim):
    """Computed the Manhatten metric of the first dim  dimesions between two numpy arrays"""
    diff = array_1[0:dim-1]-array_2[0:dim-1]    
    return numpy.linalg.norm(diff,1)

    