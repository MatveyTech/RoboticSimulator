# -*- coding: utf-8 -*-
"""
Created on Sat Nov  9 21:38:18 2019

@author: rzhavm2
"""
import numpy as np
#import Kinematics
from Kinematics import *

r_links = np.array([[5,5,5,5],
                    [0,0,0,0],
                    [0,0,0,0]])

w=np.array([[0,0,0,0],
            [0,0,0,0],
            [1,1,1,1]])

target = np.array([0,20,0])

res_theta,num_of_it = IK_4_Newthon(r_links,w,target)
    
print ("\nNumber of iterations :", num_of_it) 
print (np.rad2deg(res_theta)%360)
print ("FK : ", FK_4(r_links,w,res_theta))