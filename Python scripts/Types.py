# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 00:02:36 2020

@author: rzhavm2
"""

class Variables:
    def __init__(self, nj,npts,data): 
        
        self.nJ = nj
        self.nP = npts
        self.nE = 3
        self.bS = nj + self.nE #blockSize
        if data.shape[0] != (self.nJ+self.nE)*self.nP :
            raise "Variables class: Can't initialize variables class. Bad data dimentions!!!"
        #self.data = np.zeros((self.nJ+self.nE)*self.nP)
        self.data = data
        
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