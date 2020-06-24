# -*- coding: utf-8 -*-
"""
Created on Mon May 25 18:41:49 2020

@author: rzhavm2
"""
import numpy as np
from Kinematics import FK, CalcJacobian
from math_utils import norm_2
from CompitabilityObjective import CompitabilityObj

class IK_Obj:
    def __init__(self,links,axes, tgt,useA = False,w=1):
        self.tgt = tgt
        self.links = links
        self.axes = axes 
        self.weight = w
        self.UseAMatrix = useA
        self.co  = CompitabilityObj(0,0,links,axes,True,1)
        
    def Calc_A_Matrix(self,teta):        
        A,M = self.co.CalcAandBImproved(self.links,self.axes,self.tgt,teta) 
        return A
        
    def ComputeValue(self,curr):
        forwardK = FK(self.links,self.axes,curr)
        res = norm_2(forwardK-self.tgt)
        return res * 0.5
    
    def AddEstimatedGradientTo(self,curr,grad):
        dp = 1e-6
        for i in range(curr.shape[0]):
            tmpVal = curr[i]
            curr[i] = tmpVal + dp            
            f_P = self.ComputeValue(curr)		
            curr[i] = tmpVal - dp
            f_M = self.ComputeValue(curr)
            curr[i] = tmpVal
            res = (f_P - f_M)/(2*dp)
            grad[i] += res
    
    def AddGradientTo(self,curr,grad):
        UseEstimatedMethod = False
        if UseEstimatedMethod:
            self.AddEstimatedGradientTo(curr,grad)
        else:           
            J = CalcJacobian(self.links,self.axes,curr) 
            forwardK = FK(self.links,self.axes,curr)
            gr = np.dot(np.transpose(J),(forwardK-self.tgt)) * self.weight
            grad += gr
    
    def AddHessianTo(self,curr,hess):
        if self.UseAMatrix:
            hess += self.Calc_A_Matrix(curr)
            return
        else:
            J = CalcJacobian(self.links,self.axes,curr) 
            JT = np.transpose(J)
            JTJ = np.dot(JT,J)
            hess += JTJ
        
    def CalculateGradient(self,p):
        grad = np.zeros(p.shape[0],np.float64)
        self.AddGradientTo(p,grad)
        return grad
    
    def CalculateHessian(self,p):
        hess_shape = (p.shape[0],p.shape[0])
        hess = np.zeros(hess_shape,np.float64)
        self.AddHessianTo(p,hess)
        return hess#.reshape(p.shape,p.shape)
    
    
    
    def TestGradientWithFD(self,curr):
        analytic_gr = np.zeros(curr.shape[0],dtype='f')
        fd_gr = np.zeros(curr.shape[0],dtype='f')
        self.AddGradientTo(curr,analytic_gr)
        self.AddEstimatedGradientTo(curr,fd_gr)
#        print("Analytic grad: ", analytic_gr)
#        print("FD grad: ", fd_gr)
#        print()
        na = norm_2(analytic_gr)
        nf = norm_2(fd_gr)
        if np.abs(na) < 1e-10:
            if np.abs(nf) > 1e-10:
                print("B A D  (Zero gradient) !! Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))                
            else:
                print("Both gradients are 0")
        elif ((na-nf)/na < 1e-5):
            pass
            print("Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
        else:
            print("B A D  !! Objective Function: testing gradients...norms: analytic: {0:9.8f}, FD: {1:9.8f}.".format(na,nf))
            print("An gr:",analytic_gr)
            print("FD gr",fd_gr)
            print((na-nf)/na)
            print(analytic_gr-fd_gr)
            #print(fd_gr)