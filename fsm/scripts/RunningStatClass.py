#!/usr/bin/env python#
# A set of utility functions for converting msgs to numpy arrays, computing distance metrics 

#import rospy
#import ros
import numpy
import math
import collections 


class RunningStatClass():
    def __init__(self,str_name,queuelength):
        self.m_n            = 0
        self.m_data         = collections.deque([],queuelength)
        self.m_oldMean      = 0.0
        self.m_newMean      = 0.0
        self.m_oldSTD       = 0.0 #Standard Deviation and Variance are NOT computed for moving window!! currently incoporating all history
        self.m_newSTD       = 0.0
        self.m_queuelength  = queuelength
        self.m_name         = str_name

    def clear(self):
        self.m_n = 0
        self.m_data.clear()

    def push(self,x):
        self.m_data.append(x) 
        print "Data:", self.m_data       
        if self.m_n < self.m_queuelength:
            self.m_n += 1
            if self.m_n == 1 :
                self.m_oldMean = self.m_newMean = x
                self.m_oldSTD  = 0  
            else:
                self.m_newMean = self.m_oldMean + (x-self.m_oldMean) / self.m_n
                self.m_newSTD  = self.m_oldSTD  + (x-self.m_oldMean) * (x - self.m_newMean)
        else: #queue length reached maximum
            self.m_newMean = self.m_oldMean + (x - self.m_data[0])/self.m_n
            self.m_data.popleft() #remove oldest entry
        #print "New Mean ", self.m_newMean
        #Set up for next iteration
        print "Data:", self.m_data       
        self.m_oldMean = self.m_newMean
        self.m_oldSTD  = self.m_newSTD

    def NumDataValues(self):
        return self.m_n

    def Mean(self):
        return self.m_newMean if self.m_n>0 else 0.0

    def Variance(self):
        return self.m_newSTD/(self.m_n - 1) if self.m_n>1 else 0.0

    def StandardDeviation(self):
        return math.sqrt(self.Variance())


def main():
    R = RunningStatClass('x',4)
    print "Data :" ,R.m_data
    print "Number of elements" , R.m_n
    print "Mean : ", R.Mean()

    R.push(1.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()

    R.push(2.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()

    R.push(3.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()

    R.push(4.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()

    R.push(5.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()
    
    R.push(6.0)
    print "Number of elements" , R.m_n    
    print "Data :" ,R.m_data
    print "Mean : ", R.Mean()

    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass