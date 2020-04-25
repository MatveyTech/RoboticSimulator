# -*- coding: utf-8 -*-
"""
Created on Sun Mar 29 00:02:36 2020

@author: rzhavm2
"""
import numpy as np
import copy 

class Variables:
    
    def VerifyThetaInRange(self):
        pass
#        f = lambda x: x % (2*np.pi)
#        for i in range(self.nP):
#            t = f(self.GetTheta(i))            
#            blockInd = i*self.bS
#            self.data[blockInd:blockInd+self.nJ] = t
            
            
    def __init__(self, nj,npts,data=None): 
        
        self.nJ = nj
        self.nP = npts
        self.LastIndex = npts - 1
        self.nE = 3
        self.bS = nj + self.nE #blockSize        
        if data is None:
            data = np.zeros(self.bS*self.nP,np.float64)
        if data.shape[0] != self.bS*self.nP :
            raise "Variables class: Can't initialize variables class. Bad data dimentions!!!"
        #self.data = np.zeros((self.nJ+self.nE)*self.nP)
        self.data = data
        self.shape = data.shape[0]
        self.VerifyThetaInRange()        
    
     
    def SetData(self, newData):
        if newData.shape != self.data.shape:
            raise "new data should be the same shape as the old one"
        self.data = newData
        self.VerifyThetaInRange()
        
    
    def GetVal(self,i,j):
        return self.data[i*self.bS + j]
    
    
    def SetVal(self,i,j,val):
        self.data[i*self.bS + j] = val
        self.VerifyThetaInRange()
    
    
    def AddVal(self,i,j,val):
        self.data[i*self.bS + j] = self.data[i*self.bS + j] + val
        self.VerifyThetaInRange()
        
    
    def GetTheta(self,i):
        blockInd = i*self.bS
        return self.data[blockInd:blockInd+self.nJ]
    
    
    def SetTheta(self,i,t):
        if t.shape[0] != self.nJ :
            raise "Variables class: Bad Tetha dimentions!!!"
        blockInd = i*self.bS
        self.data[blockInd:blockInd+self.nJ] = t
        self.VerifyThetaInRange()
    
    
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
    
    def Copy(self):
        return copy.deepcopy(self)  
    
#    def __getitem__(self, key):
#        return self.data[key]
#v.data[0]=6
#v.data[5]=7.2
#v.data[6]=-0.5
#print(v)
#v.VerifyThetaInRange()
#print(v)