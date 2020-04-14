# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 16:38:59 2020

@author: Matvey Rzhavskiy
"""

from Types import Variables
from math_utils import norm_2

class GradientBasedFunctionMinimizer:
    
    def computeSearchDirection(self):
        raise "GradientBasedFunctionMinimizer is a abstract class"
    
    def minimize(self,objective,p):
        
        maxIterations = 1
        optimizationConverged = False
        currentPoint = p.Copy()
        for i in range(0,maxIterations):
            searchDirection = self.computeSearchDirection(objective,currentPoint)
            if norm_2(searchDirection) < 0.00001:
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
        
    def computeSearchDirection(self):
        print("computeSearchDirection GradientDescentFunctionMinimizer")

class NewtonFunctionMinimizer(GradientBasedFunctionMinimizer):
        
    def computeSearchDirection(self):
        print("computeSearchDirection NewtonFunctionMinimizer")
        
#x = NewtonFunctionMinimizer()
#x.minimize(None)
#
#x1 = pi = Variables(curr.nJ,curr.nP,newData)
    