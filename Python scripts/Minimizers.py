# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 16:38:59 2020

@author: Matvey Rzhavskiy
"""

from Types import Variables
from math_utils import norm_2
from math_utils import epsilon
from math_utils import AddEpsToDiagonal
from math_utils import norm_2, scalar_prod,normalize
import numpy as np
import copy

class GradientBasedFunctionMinimizer:
    
    def __init__(self,maxIterations=1,isVar=True):
        self.maxiterations = maxIterations
        self.isVar = isVar
        
    
    def computeSearchDirection(self,regFactor):
        raise "GradientBasedFunctionMinimizer is a abstract class"
    
    def minimize(self,objective,p,regFactor):
        
        optimizationConverged = False
        #currentPoint = p.Copy()
        currentPoint = copy.deepcopy(p)  
        for i in range(0,self.maxiterations):
            searchDirection = self.computeSearchDirection(objective,currentPoint,regFactor)
            #print("norm_2(searchDirection):",norm_2(searchDirection))
            if norm_2(searchDirection) < epsilon:
                optimizationConverged = True
                break
            alpha,currentPoint = self.doLineSearch(objective,currentPoint,searchDirection)
        
        return currentPoint, optimizationConverged
    
    
    def doLineSearch(self,objective,curr,searchDir):
        if self.isVar:
            return self.doLineSearchVar(objective,curr,searchDir)
        else:
            return self.doLineSearchA(objective,curr,searchDir)
    
    def doLineSearchA(self,objective,curr,searchDir): #bad code, the input here is regular nparray
        lineSearchStartValue = 1
        maxNumOfIterations = 15
        
        alpha = lineSearchStartValue
        initialObjValue = objective.ComputeValue(curr)
        
        pi = None
        
        for i in range(0,maxNumOfIterations):
            pi = curr.data + searchDir * alpha
            newObjValue = objective.ComputeValue(pi)
            if newObjValue < initialObjValue:
                return alpha,pi
            else:
                alpha = alpha / 2
        
        
        grad = objective.CalculateGradient(curr)
        isDescent = self.isSearchDirectionDescent(searchDir,grad)
        print("line search failed! Is descent direction?",isDescent)
        
        return alpha,pi    
    
    def isSearchDirectionDescent(self,searchDir,grad):
        return scalar_prod(normalize(searchDir),normalize(grad)) < 0
    
    def calcAngleBetween(self,searchDir,grad):
        return scalar_prod(normalize(searchDir),normalize(grad)) 
    
    def doLineSearchVar(self,objective,curr,searchDir): #bad code, the input here is type of Var
        lineSearchStartValue = 1
        maxNumOfIterations = 15
        
        alpha = lineSearchStartValue
        initialObjValue = objective.ComputeValue(curr)
        
        pi = None
        
        for i in range(0,maxNumOfIterations):
            newData = curr.data + searchDir * alpha
            pi = Variables(curr.nJ,curr.nP,newData)
            newObjValue = objective.ComputeValue(pi)
            if newObjValue < initialObjValue:
                return alpha,pi
            else:
                alpha = alpha / 2
        
        grad = objective.CalculateGradient(curr)
        isDescent = self.isSearchDirectionDescent(searchDir,grad)
        print("line search failed! Is descent direction?",isDescent)
        
        return alpha,pi    
    


class GradientDescentFunctionMinimizer(GradientBasedFunctionMinimizer):

    def __init__(self,isVarValue=True):
        GradientBasedFunctionMinimizer.__init__(self, isVar=isVarValue)
    
    def GetName(self):
        return "Gradient Descent Minimizer"
        
    def computeSearchDirection(self,objective,p,regFactor):
        grad = np.zeros(p.data.shape,np.float64)
        objective.AddGradientTo(p,grad)
        return -grad

class NewtonFunctionMinimizer(GradientBasedFunctionMinimizer):
    
    def __init__(self,isVarValue=True):
        GradientBasedFunctionMinimizer.__init__(self, isVar=isVarValue)
    
    def GetName(self):
        return "Newton Minimizer"    
    
    def computeSearchDirection(self,objective,p,regFactor):
        grad = objective.CalculateGradient(p)
        hess = objective.CalculateHessian(p)                
        AddEpsToDiagonal(hess,regFactor)
        dp = np.dot(np.linalg.pinv(hess),grad)
        return -dp
        
        
#x = GradientDescentFunctionMinimizer()
#y = x.computeSearchDirection(None,np.array([1,2,3,4]))
#print(y)
#x.minimize(None)
#
#x1 = pi = Variables(curr.nJ,curr.nP,newData)
    