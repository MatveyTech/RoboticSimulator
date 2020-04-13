# -*- coding: utf-8 -*-
"""
Created on Sat Apr 11 16:38:59 2020

@author: Matvey Rzhavskiy
"""

from Types import Variables

class GradientBasedFunctionMinimizer:
    
    def minimize(self):
        print("Base minimize is here")
    
    
    def doLineSearch(self,objective,curr,searchDir):
        lineSearchStartValue = 1
        maxNumOfIterations = 15
        
        alpha = lineSearchStartValue
        initialObjValue = objective.ComputeValue(curr)
        
        newObjValue = None
        
        for i in range(0,maxNumOfIterations):
            newData = curr.data - searchDir * alpha
            pi = Variables(curr.nJ,curr.nP,newData)
            newObjValue = objective.ComputeValue(pi)
            if newObjValue < initialObjValue:
                return alpha,newObjValue
            else:
                alpha = alpha / 2
        
        print("line search failed!")
        return alpha,newObjValue
    
    


class GradientDescentFunctionMinimizer(GradientBasedFunctionMinimizer):
        
    def doLineSearch(self):
        print("GradientDescentFunctionMinimizer doLineSearch is here")

class NewtonFunctionMinimizer(GradientBasedFunctionMinimizer):
        
    def doLineSearch(self):
        pass
    