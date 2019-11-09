# -*- coding: utf-8 -*-
"""
Created on Sat Nov  9 21:11:11 2019

@author: rzhavm2
"""
import Kinematics
import numpy as np

links = np.array([[5,0],
                  [0,6],
                  [0,0]])
w = np.array([
        [0,1],
        [0,0],
        [1,0]
        ])

#this one is input for FK
theta = np.array([np.deg2rad(0),np.deg2rad(45)])
ee = FK_2(links,w,theta)
print(ee)