# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 00:02:36 2020

@author: rzhavm2
"""
import numpy as np

class Variables:
    def __init__(self, nj,npts,data=None): 
        
        self.nJ = nj
        self.nP = npts
        self.LastIndex = npts - 1
        self.nE = 3
        self.bS = nj + self.nE #blockSize
        if data is None:
            data = np.zeros(self.bS*self.nP)
        if data.shape[0] != self.bS*self.nP :
            raise "Variables class: Can't initialize variables class. Bad data dimentions!!!"
        #self.data = np.zeros((self.nJ+self.nE)*self.nP)
        self.data = data
        
    def GetVal(self,i,j):
        return self.data[i*self.bS + j]
    
    def SetVal(self,i,j,val):
        self.data[i*self.bS + j] = val
    
    def AddVal(self,i,j,val):
        self.data[i*self.bS + j] = self.data[i*self.bS + j] + val
        
    def GetTheta(self,i):
        blockInd = i*self.bS
        return self.data[blockInd:blockInd+self.nJ]
    
    def SetTheta(self,i,t):
        if t.shape[0] != self.nJ :
            raise "Variables class: Bad Tetha dimentions!!!"
        blockInd = i*self.bS
        self.data[blockInd:blockInd+self.nJ] = t
    
    def GetEE(self,i):        
        blockInd = i*self.bS
        return self.data[blockInd+self.nJ:blockInd+self.nJ+self.nE]
    
    def SetEE(self,i,ee):
        if ee.shape[0] != self.nE :
            raise "Variables class: Bad EE dimentions!!!"
        blockInd = i*self.bS
        self.data[blockInd+self.nJ:blockInd+self.nJ+self.nE] = ee
        
    def AddEE(self,i,ee):
        if ee.shape[0] != self.nE :
            raise "Variables class: Bad EE dimentions!!!"
        self.SetEE(i,self.GetEE(i)+ee)    
    

    def GetFirstEEInd(self):
        return self.nJ
    
    def GetLastEEInd(self):
        return self.bS*self.nP - self.nE
    
    
    def __str__(self):
        x = np.reshape(self.data, (self.nP,self.bS))
        return x.__str__()