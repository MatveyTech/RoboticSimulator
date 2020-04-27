# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 16:38:59 2020

@author: Matvey Rzhavskiy
"""

from Types import Variables
from math_utils import norm_2
from math_utils import epsilon
from math_utils import AddEpsToDiagonal
import numpy as np

class GradientBasedFunctionMinimizer:
    
    def __init__(self,maxIterations=1):
        self.maxiterations = maxIterations
        
    
    def computeSearchDirection(self):
        raise "GradientBasedFunctionMinimizer is a abstract class"
    
    def minimize(self,objective,p):
        
        optimizationConverged = False
        currentPoint = p.Copy()
        for i in range(0,self.maxiterations):
            searchDirection = self.computeSearchDirection(objective,currentPoint)
            if norm_2(searchDirection) < epsilon:
                optimizationConverged = True
                break
            alpha,currentPoint = self.doLineSearch(objective,currentPoint,searchDirection)
        
        return currentPoint, optimizationConverged
    
    
    def doLineSearch(self,objective,curr,searchDir):
        lineSearchStartValue = 1
        maxNumOfIterations = 15
        
        alpha = lineSearchStartValue
        initialObjValue = objective.ComputeValue(curr)
        
        pi = None
        
        for i in range(0,maxNumOfIterations):
            newData = curr.data - searchDir * alpha
            pi = Variables(curr.nJ,curr.nP,newData)
            newObjValue = objective.ComputeValue(pi)
            if newObjValue < initialObjValue:
                return alpha,pi
            else:
                alpha = alpha / 2
        
        print("line search failed!")
        return alpha,pi    
    


class GradientDescentFunctionMinimizer(GradientBasedFunctionMinimizer):
    
    def GetName(self):
        return "Gradient Descent Minimizer"
        
    def computeSearchDirection(self,objective,p):
        grad = np.zeros(p.data.shape,np.float64)
        objective.AddGradientTo(p,grad)
        return grad

class NewtonFunctionMinimizer(GradientBasedFunctionMinimizer):
    
    def GetName(self):
        return "Newton Minimizer"    
    
    def computeSearchDirection(self,objective,p):
        grad = np.zeros(p.data.shape,np.float64)
        objective.AddGradientTo(p,grad)
        
        hess_shape = (p.data.shape[0],p.data.shape[0])
        hess = np.zeros(hess_shape,np.float64)
        objective.AddHessianTo(p,hess)
        AddEpsToDiagonal(hess,0.01)
        dp = np.dot(np.linalg.inv(hess),grad)
        return dp
        
        
#x = GradientDescentFunctionMinimizer()
#y = x.computeSearchDirection(None,np.array([1,2,3,4]))
#print(y)
#x.minimize(None)
#
#x1 = pi = Variables(curr.nJ,curr.nP,newData)
    