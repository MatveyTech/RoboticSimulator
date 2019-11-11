# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 08:56:50 2019

@author: rzhavm2
"""

import numpy as np
#import Kinematics
from Kinematics import *

def BuildNew2JointsRobot(links,w,teta1,teta2):
    """    
    This function works only for 2D now!!!!!
    """
    offset = np.array([0,0,0])
    for i in range(1,teta1):
        #print (i)
        offset = offset + links[:,i-1]
    end_of_first_link = np.array([0,0,0])
    #print ("\nThe offset :", offset) 
    for i in range(teta1,teta2):
        #print (i)
        end_of_first_link = end_of_first_link + links[:,i-1]
    end_of_second_link = np.array([0,0,0])
    #print ("\nEnd of first link :", end_of_first_link) 
    for i in range(teta2,links.shape[1]+1):
        #print (i)
        end_of_second_link = end_of_second_link + links[:,i-1]    
    #print ("\nEnd of second link :", end_of_second_link) 
    #return end_of_first_link,end_of_second_link
    return offset, np.column_stack((end_of_first_link,end_of_second_link))

r_links = np.array([[5,5,5,5],
                    [0,0,0,0],
                    [0,0,0,0]])

w=np.array([[0,0,0,0],
            [0,0,0,0],
            [1,1,1,1]])

target = np.array([0,20,0])

offset,new_links = BuildNew2JointsRobot(r_links,None,2,4)

print ("Old links :") 
print (r_links)
print ("New links :") 
print (new_links)
print ("Offset :") 
print (offset)
