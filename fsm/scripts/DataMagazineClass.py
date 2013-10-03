#!/usr/bin/env python#
# A set of utility functions for converting msgs to numpy arrays, computing distance metrics 

#import rospy
#import ros
import numpy
import math
import collections 
from array import array


class DataMagazineClass():
    def __init__(self,str_name,queuelength):
        self.m_n            = 0
        self.m_data         = collections.deque([],queuelength)
        self.m_queuelength  = queuelength
        self.m_name         = str_name

    def clear(self):
        self.m_n = 0
        self.m_data.clear()

    def push(self,x):
        self.m_data.append(x)         
        if self.m_n < self.m_queuelength : 
            self.m_n += 1

    def NumDataValues(self):
        return self.m_n

    def Mean_Variance(self):
        mean_old = mean_new = 0
        var = 0
        for i in range(self.m_n):
            mean_new = mean_old + (self.m_data[i]-mean_old) / (i+1)            
            var +=(self.m_data[i]-mean_old)*(self.m_data[i]-mean_new)
            mean_old = mean_new #setup for next iteration
        return (mean_new,var)
        
    def Variance(self):
        var = 0
        for i in range(self.m_n):
            var += (self.m_data[i]-mean) / (i+1)            
        return mean
        
        
        return self.m_newSTD/(self.m_n - 1) if self.m_n>1 else 0.0

    def StandardDeviation(self):
        return math.sqrt(self.Variance())


def main():
    R = RunningStatClass('x',4)    
##    stream = array('d',[1.0 ,2.0,3.0,4.0 ,5.0,6.0 ])
    stream = array('d',[1.0 ,1.0,1.0,1.0 ,1.0,6.0 ])
    
    for i in range(0,len(stream)):
        print "Pushed" 
        R.push(stream[i])        
        print "Data :" ,R.m_data
        print "Number of elements" , R.m_n
        print "Mean, Variance : ", R.Mean_Variance(),"\n\n"
   
if __name__ == '__main__':
##    try:
        main()
##    except rospy.ROSInterruptException: pass