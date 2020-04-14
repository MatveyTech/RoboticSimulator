# -*- coding: utf-8 -*-
"""
Created on Sat Mar 28 21:53:56 2020

@author: rzhavm2
"""

from Types import Variables
from math_utils import norm_2
import numpy as np
from CompitabilityObjective import CompitabilityObj

  

class SmoothnessObj:
    def __init__(self,nj,npoints,w=1):
        self.weight = w
        ne = 3
        blockSize = ne+nj
        c = blockSize * npoints
        r = c - blockSize - nj
        self.A = np.zeros((r,c))
        for i in range(0,r):
            self.A[i,i+nj]=-1
            self.A[i,i+nj+blockSize]=1        
        self.ATA = np.dot(np.transpose(self.A),self.A)
        
    def ComputeValue(self,curr):
        m = np.dot(self.A,curr.data)
        return norm_2(m) * self.weight
        
    
    def AddGradientTo(self,curr,grad):
        tres = 2 * self.weight * np.dot(self.ATA,curr)
        grad += tres
        
    
    def ComputeHessian(self,curr,hess):
        res = 2 * self.weight * self.ATA
        hess += res
    

class StartObj:
    def __init__(self,ee_start,w=1):
        self.ee_start = ee_start
        self.weight = w
        
    def ComputeValue(self,var):
        firstPos = var.GetEE(0)
        if firstPos.shape[0] != 3 :
            raise "StartObj is working now only with the EE dimention of 3!"
        diff = firstPos - self.ee_start
        return (norm_2(diff)) * self.weight
    
    def AddGradientTo(self,curr,grad):
        firstPos = curr.GetEE(0)
        grad_val = 2 *(firstPos - self.ee_start)*self.weight
        tgrad = Variables(curr.nj,curr.npts)
        tgrad.SetEE(0, grad_val)
        grad  += tgrad.data  
    
    def AddHessianTo(self,curr,hess):
        fr = curr.GetFirstEEInd()
        to = curr.GetFirstEEInd() + curr.nE
        for x in range(fr,to):
            hess[x,x] = hess[x,x] + 2
    
    
class FinalObj:
    def __init__(self,ee_final,w=1):
        self.ee_final = ee_final
        self.weight = w
        
    def ComputeValue(self,curr):
        lastPos = curr.GetEE(curr.LastIndex)
        if lastPos.shape[0] != 3 :
            raise "FinalObj is working now only with the EE dimention of 3!"
        diff = lastPos - self.ee_final
        return (norm_2(diff)) * self.weight
    
    def AddGradientTo(self,curr,grad):
        lastPos = curr.GetEE(curr.LastIndex)
        grad_val = 2 *(lastPos - self.ee_final)*self.weight
        tgrad = Variables(curr.nj,curr.npts)
        tgrad.SetEE(curr.LastIndex, grad_val)  
        grad  += tgrad.data  
    
    def AddHessianTo(self,curr,hess):
        fr = curr.GetLastEEInd()
        to = curr.GetLastEEInd() + curr.nE
        for x in range(fr,to):
            hess[x,x] = hess[x,x] + 2


class TheObjective:
    def __init__(self,ee_start,ee_final,nj,npts,links,axes):
        self.StartObjective = StartObj(ee_start)
        self.FinalObjective = FinalObj(ee_final)
        self.SmoothnessObjective = SmoothnessObj(nj,npts)
        self.CompitabilityObjective  = CompitabilityObj(nj,npts,links,axes)
        self.AllObjectives = [self.StartObjective,self.FinalObjective,self.SmoothnessObjective,self.CompitabilityObjective]
        
    def ComputeValue(self,curr):
        res = 0
        for obj in self.AllObjectives: 
            res += obj.ComputeValue(curr)
        return res
    
    def AddGradientTo(self,curr,grad):
        for obj in self.AllObjectives: 
            obj.AddGradientTo(curr,grad)
    
    
#so = StartObj(np.array([0,0,0]))
#
#from numpy import linalg as LA
#import numpy as np
#from Types import Variables
#
#def f(v):
#    v.SetEE(0,v.GetEE(0) + np.array([1,2,3]))
#    
#v = Variables(3,10)
#v.SetTheta(0,np.array([10,20,30]))
#v.SetEE(9,np.array([100,200,300]))
#print (v.GetLastEEInd())
           

so = FinalObj(np.array([0,0,0]))
v = Variables(3,10)
hess = np.zeros((60,60))
#print(v)
so.AddHessianTo(v,hess)
#print(hess)

v.SetEE(0,np.array([5,5,5]))
v.SetEE(2,np.array([1,2,3]))
sm_o = SmoothnessObj(3,10)


li = np.array([[200, 150, 250],[0,0,0],[0,0,0]])
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])

v = Variables(3,10)
co = CompitabilityObj(3,10,li,ax)
o = TheObjective(np.array([0,0,0]),np.array([0,0,0]),3,10,li,ax)
print(o.ComputeValue(v))



