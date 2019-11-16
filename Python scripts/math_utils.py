# -*- coding: utf-8 -*-
"""
Created on Wed Nov  6 10:31:51 2019

@author: rzhavm2
"""
import numpy as np
from numpy.linalg import norm

comparisonFloat = 0.05
epsilon = 0.000001
smallFloat = 0.001

def CreateRotationMatrix(teta,u):
    """    
    Returns rotation matrix to rotate teta readians around u
    """
    [x,y,z] = u
    c = np.cos(teta)
    s = np.sin(teta)
    res = np.array([[c+x*x*(1-c),x*y*(1-c)-z*s,x*z*(1-c)+y*s],[y*x*(1-c)+z*s,c+y*y*(1-c),y*z*(1-c)-x*s],[z*x*(1-c)-y*s,z*y*(1-c)+x*s,c+z*z*(1-c)]]) 
    return res

def normalize(v):
    return v / norm(v)


def compare(a1,a2):
    if np.abs(a1[0]-a2[0]) < comparisonFloat and np.abs(a1[1]-a2[1]) < comparisonFloat and np.abs(a1[2]-a2[2]) < comparisonFloat:
        return True
    return False

def GetAngleBetweenVec(v1, v2, cr):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    #print ("V1,v2, before normalization",v1,v2)
    v1 = normalize(v1)
    v2 = normalize(v2)   

    if np.abs(v1[0]-v2[0]) < smallFloat and np.abs(v1[1]-v2[1]) < smallFloat and np.abs(v1[2]-v2[2]) < smallFloat:
        return 0
    #print (v1,v2)
    c = np.dot(v1,v2)
    if c < -1 and np.abs(-1 - c) < epsilon:
        c = -1
    if c > 1 and np.abs(1 - c) < epsilon:
        c = 1
    #print("C : % 5.2f" % c)  
    teta = np.arccos(c)
    R=CreateRotationMatrix(teta,cr)
    cand1 = np.dot(R,v1)
    prod1 = np.dot(cand1,v2)
    if np.abs(prod1 - 1) < epsilon:
        return teta
    else:
        teta = 2*np.pi-teta
        R=CreateRotationMatrix(teta,cr)
        cand2 = np.dot(R,v1)
        prod2 = np.dot(cand2,v2)
        if np.abs(prod2 - 1) < smallFloat:
            return teta
    raise ValueError('The vec angle function failed')
    
def Get_C_OppositeAngle(a,b,c):
    #return np.rad2deg(np.arccos((a*a+b*b-c*c)/(2*a*b)))
    if c==0:
        return 0
    return np.arccos((a*a+b*b-c*c)/(2*a*b))
    

def GetTriangleAngles(a,b,c):
    """    
    Use cosine law to calculate triangle angles
    """
    alpha = Get_C_OppositeAngle(b,c,a)
    beta = Get_C_OppositeAngle(c,a,b)
    gamma = Get_C_OppositeAngle(a,b,c)
    return (alpha,beta, gamma)