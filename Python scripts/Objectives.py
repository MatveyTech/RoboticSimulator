# -*- coding: utf-8 -*-
"""
Created on Sat Mar 28 21:53:56 2020

@author: rzhavm2
"""

from Types import Variables
import numpy as np

def norm_2(x):
    if (x.shape[0]!=3):
        raise "norm_2 is working now only with the dimention of 3!"    
    return x[0] * x[0] + x[1] * x[1] + x[2] * x[2]

class MyClass:
  def method(self, arg):
    print(arg)

class CompitabilityObj:
    def __init__(self):
        self.x = 5
        
    def ComputeValue(self,p):
        pass
    
    def ComputeGradient(self,p):
        pass
    
    def ComputeHessian(self,p):
        pass
    

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
        m = np.dot(self.A,curr)
        return norm_2(m) * self.weight
        
    
    def AddGradientTo(self,curr,grad):
        tres = 2 * self.weight * np.dot(self.ATA,curr)
        grad += tres
        
    
    def ComputeHessian(self,p):
        pass
    

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
        grad.AddEE(0, grad_val)        
    
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
        lastPos = curr.GetEE(var.LastIndex)
        if lastPos.shape[0] != 3 :
            raise "FinalObj is working now only with the EE dimention of 3!"
        diff = lastPos - self.ee_final
        return (norm_2(diff)) * self.weight
    
    def AddGradientTo(self,curr,grad):
        lastPos = curr.GetEE(curr.LastIndex)
        grad_val = 2 *(lastPos - self.ee_final)*self.weight
        grad.AddEE(curr.LastIndex, grad_val)        
    
    def AddHessianTo(self,curr,hess):
        fr = curr.GetLastEEInd()
        to = curr.GetLastEEInd() + curr.nE
        for x in range(fr,to):
            hess[x,x] = hess[x,x] + 2
    
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
#print(sm_o.ComputeValue(v))

#from tkinter import *
#
#def show_values():
#    print (w1.get(), w2.get())
#
#master = Tk()
#w1 = Scale(master, from_=0, to=42)
#w1.pack()
#w2 = Scale(master, from_=0, to=200, orient=HORIZONTAL)
#w2.pack()
#Button(master, text='Show', command=show_values).pack()
#
#mainloop()
#print("HI")
