 #!/usr/bin/env python#


import numpy
import rospy
import ros

    
#UTILITY FUNCTION
def PoseMsg2NumpyArray(pose_msg):
    v =  numpy.array([pose_msg.position.x,
                    pose_msg.position.y,
                    pose_msg.position.z,
                    pose_msg.orientation.x,
                    pose_msg.orientation.y,
                    pose_msg.orientation.z,
                    pose_msg.orientation.w])
    return v

def PoseMsg2NumpyArrayPosition(pose_msg):
   v =  PoseMsg2NumpyArray(pose_msg)
   v = v[0:3]
   return v

def PointMsg2NumpyArrayPosition(point_msg):
   v =  numpy.array([point_msg.x,
                    point_msg.y,
                    point_msg.z])  
   return v
    

def Distance(str_metric, array_1, array_2):
        if str_metric is 'Manhatten':
            return ManhattenPlanarDistance(array_1, array_2)
        else:
            return EuclideanPlanarDistance(array_1, array_2)

def EuclideanPlanarDistance( array_1, array_2):
    diff = array_1[0:2]-array_2[0:2]    
    return numpy.linalg.norm(diff,2)


def ManhattenPlanarDistance( array_1, array_2):
    diff = array_1[0:2]-array_2[0:2]    
    return numpy.linalg.norm(diff,1)

    