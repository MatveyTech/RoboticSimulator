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
        self.A = np.zeros((r,c),dtype='f')
        for i in range(0,r):
            self.A[i,i+nj]=-1
            self.A[i,i+nj+blockSize]=1        
        self.ATA = np.dot(np.transpose(self.A),self.A)
        
    def ComputeValue(self,curr):
        m = np.dot(self.A,curr.data)
        return norm_2(m) * self.weight
        
    
    def AddGradientTo(self,curr,grad):
        tres = 2 * self.weight * np.dot(self.ATA,curr.data)
        grad += tres
        
    
    def AddHessianTo(self,curr,hess):
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
        tgrad = Variables(curr.nJ,curr.nP)
        tgrad.SetEE(0, grad_val)
        grad  += tgrad.data  
    
    def AddHessianTo(self,curr,hess):
        fr = curr.GetFirstEEInd()
        to = curr.GetFirstEEInd() + curr.nE
        for x in range(fr,to):
            hess[x,x] = hess[x,x] + 2 * self.weight
    
    
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
        tgrad = Variables(curr.nJ,curr.nP)
        tgrad.SetEE(curr.LastIndex, grad_val)  
        grad  += tgrad.data  
    
    def AddHessianTo(self,curr,hess):
        fr = curr.GetLastEEInd()
        to = curr.GetLastEEInd() + curr.nE
        for x in range(fr,to):
            hess[x,x] = hess[x,x] + 2 * self.weight


class TheObjective:
    def __init__(self,ee_start,ee_final,nj,npts,links,axes,weights):
        self.StartObjective = StartObj(ee_start,weights[0])
        self.FinalObjective = FinalObj(ee_final,weights[1])
        self.SmoothnessObjective = SmoothnessObj(nj,npts,weights[2])
        self.CompitabilityObjective  = CompitabilityObj(nj,npts,links,axes,weights[3])
        self.AllObjectives = []
        self.AllObjectives.append(self.StartObjective)
        self.AllObjectives.append(self.FinalObjective)
        self.AllObjectives.append(self.SmoothnessObjective)
        self.AllObjectives.append(self.CompitabilityObjective)
        
               
    def ComputeValue(self,curr):
        res = 0
        for obj in self.AllObjectives: 

            res += obj.ComputeValue(curr)
        return res
    
    def AddGradientTo(self,curr,grad):
        for obj in self.AllObjectives: 
            obj.AddGradientTo(curr,grad)
            
    def AddHessianTo(self,curr,hess):
        for obj in self.AllObjectives: 
            obj.AddHessianTo(curr,hess)
            
    def AddEstimatedGradientTo(self,curr,grad):
        dp = 1e-6
        data = curr.data
        for i in range(curr.shape):
            tmpVal = data[i]
            data[i] = tmpVal + dp            
            f_P = self.ComputeValue(curr)		
            data[i] = tmpVal - dp
            f_M = self.ComputeValue(curr)
            data[i] = tmpVal
            res = (f_P - f_M)/(2*dp)
            grad[i] += res
    
    def TestGradientWithFD(self,curr):
        analytic_gr = np.zeros(curr.data.shape,dtype='f')
        fd_gr = np.zeros(curr.data.shape,dtype='f')
        self.AddGradientTo(curr,analytic_gr)
        self.AddEstimatedGradientTo(curr,fd_gr)
        na = norm_2(analytic_gr)
        nf = norm_2(fd_gr)
        if np.abs(na) < 1e-10:
            if np.abs(nf) > 1e-10:
                print("B A D  (Zero gradient) !! Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))                
            else:
                print("Both gradients are 0")
        elif ((na-nf)/na < 1e-5):
            pass
            #print("Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
        else:
            print("B A D  !! Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
            print((na-nf)/na)
            print(analytic_gr-fd_gr)
            #print(fd_gr)
#        
            
            
    def AddEstimatedHessianTo(self,curr,hess):
        dp = 1e-5
        data = curr.data
        for i in range(curr.shape):
            C_P = np.zeros(data.shape,np.float64)
            C_M = np.zeros(data.shape,np.float64)
            tmpVal = data[i]
            data[i] = tmpVal + dp
            self.AddGradientTo(curr,C_P)
            data[i] = tmpVal - dp
            self.AddGradientTo(curr,C_M)  
            data[i] = tmpVal
            res = (C_P - C_M)/(2*dp)
            hess[i] += res
            
    
    def TestHessianWithFD(self,curr):
        hess_shape = (curr.data.shape[0],curr.data.shape[0])
        analytic_hess = np.zeros(hess_shape,np.float64)
        fd_hess = np.zeros(hess_shape,np.float64)
        self.AddHessianTo(curr,analytic_hess)
        self.AddEstimatedHessianTo(curr,fd_hess)         
#        size = 12
#        print(analytic_hess[0:size,0:size])
#        print(fd_hess[0:size,0:size])
        na = norm_2(analytic_hess)
        nf = norm_2(fd_hess)
        
        if np.abs(na) < 1e-10:
            if np.abs(nf) > 1e-10:
                print("B A D  (Zero hessian) !! Objective Function: testing hessians...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))                
            else:
                print("Both hessians are 0")
        elif ((na-nf)/na < 1e-5):
            #pass
            print("Objective Function: testing hessians...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
        else:
            print("B A D  !! Objective Function: testing hessians...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
            print((na-nf)/na)
            #print(analytic_gr-fd_gr)
            #print(fd_gr)
        #print ("The hessian error is:",(na-nf)/na)
            

#x = np.array([8e-2,6,7,7])
#print (x.shape[0])
#for i in x:
#    i = i+1
#    
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
           

#so = FinalObj(np.array([0,0,0]))
#v = Variables(3,10)
#hess = np.zeros((60,60))
##print(v)
#so.AddHessianTo(v,hess)
##print(hess)
#
#v.SetEE(0,np.array([5,5,5]))
#v.SetEE(2,np.array([1,2,3]))
#sm_o = SmoothnessObj(3,10)
#
#
#li = np.array([[200, 150, 250],[0,0,0],[0,0,0]])
#ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
#
#v = Variables(3,10)
#co = CompitabilityObj(3,10,li,ax)
#o = TheObjective(np.array([0,0,0]),np.array([0,0,0]),3,10,li,ax)
#print(o.ComputeValue(v))



