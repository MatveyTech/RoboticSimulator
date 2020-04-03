# -*- coding: utf-8 -*-
"""
Created on Sat Mar 28 21:53:56 2020

@author: rzhavm2
"""

from Types import Variables

class CompitabilityObj:
    def __init__(self):
        self.x = 5
        
    def ComputeValue(p):
        pass
    
    def ComputeGradient(p):
        pass
    
    def ComputeHessian(p):
        pass
    

class SmoothnessObj:
    def __init__(self):
        self.x = 5
        
    def ComputeValue(p):
        pass
    
    def ComputeGradient(p):
        pass
    
    def ComputeHessian(p):
        pass
    

class StartObj:
    def __init__(self,ee_start,w=1):
        self.ee_start = ee_start
        self.weight = w
        
    def ComputeValue(var):
        firstPos = var.GetTheta(0)
        if firstPos.shape[0] != 3 :
            raise "StartObj is working now only with the dimention of 3!"
        r = firstPos - self.ee_start
        return (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]) * self.weight
    
    def ComputeGradient(var):
        res = create array here
        firstPos = var.GetTheta(0)
        return -2 
        
    
    def ComputeHessian(p):
        pass
    
    
class FinalObj:
    def __init__(self):
        self.x = 5
        
    def ComputeValue(p):
        pass
    
    def ComputeGradient(p):
        pass
    
    def ComputeHessian(p):
        pass
    
so = StartObj(np.array([0,0,0]))