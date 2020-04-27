# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 17:15:07 2020

@author: rzhavm2
"""


from Minimizers import GradientDescentFunctionMinimizer
from Minimizers import NewtonFunctionMinimizer
from Objectives import TheObjective
from Objectives import StartObj
from Objectives import FinalObj
from Objectives import SmoothnessObj
import numpy as np
from Types import Variables
from Kinematics import FK

#class XSquare:
#        
#    def ComputeValue(self,p):
#        x = p.data[0]
#        if x < 0:
#            return x * x
#        else:
#            return x * x * x
#    
#    def AddGradientTo(self,p,grad):
#        x = p.data[0]
#        if x < 0:
#            tgrad = 2 * x
#        else:
#            tgrad = 3 * x * x       
#        grad[0]  += tgrad  
#
#x = Variables(1,1)
#x.data[0] = 1000000000000000000000000000000
##x = np.array([10])
#
#m = GradientDescentFunctionMinimizer(1)
#obj = XSquare()
#c = False
#p = x
#while c is False:
#    p,c = m.minimize(obj,p)
#    print (p.data,c)    
    
li = np.array([[200, 150, 250],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([100,0,0],dtype='f')
end_point = np.array([-100,0,0],dtype='f')

li = np.array([[20, 15, 25],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([10,0,0],dtype='f')
end_point = np.array([-10,0,0],dtype='f')

m = GradientDescentFunctionMinimizer(1)
m = NewtonFunctionMinimizer()

numT = 3
numP = 5

obj = TheObjective(st_point,end_point,numT,numP,li,ax,np.array([1,1,1,1],dtype='f'))
#obj = SmoothnessObj(0,4)

v = Variables(numT,numP)
v.SetEE(0,np.array([100,20,30],dtype='f'))
#v.SetEE(1,np.array([100,200,300]))
#print(v)
#gr = np.zeros(v.data.shape)
#obj.AddGradientTo(v,gr)

#print(gr)
#
#x2 = np.array([10e-10,0,0])
#input(obj.ComputeValue(Variables(0,1,x2)))
c = False
p = v
#p = ps[180]
#ps = []
counter = 0
print (p)
while c is False and counter<100000:
    counter +=1
    print ("Iteration:",counter)
    #obj.TestGradientWithFD(p)
    #obj.TestHessianWithFD(p)
    p,c = m.minimize(obj,p)  
    #print(obj.ComputeValue(p))
    #ps.append(p)
    #print (p,c)
print("\n")
print (p,c)
res_str = print("\n\033[1;31;47mMinimization method: {0}. Num of iterations: {1}.".format(m.GetName(),counter))


#for i in range(p.nP):
#    t = p.GetTheta(i)
#    e = p.GetEE(i)
#    print (FK(li,ax,t)-e)

#fk = FK(li,ax, np.array([1.1440029882e+03,  7.8483575105e+02,  5.2133984492e+02]))
#print (fk)  
