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
    new_w = w
    return offset, np.column_stack((end_of_first_link,end_of_second_link)), new_w

def TestBuildNew2JointsRobot():
    r_links = np.array([[5,5,5,5],
                        [0,0,0,0],
                        [0,0,0,0]])
    
    w=np.array([[0,0,0,0],
                [0,0,0,0],
                [1,1,1,1]])
    
    target = np.array([0,20,0])
    
    offset,new_links = BuildNew2JointsRobot(r_links,None,1,4)
    
    print ("Old links :") 
    print (r_links)
    print ("New links :") 
    print (new_links)
    print ("Offset :") 
    print (offset)

def CorrectPointToRobotEnvelope2D(links,target,robot_offset = np.array([0,0,0])):
    """    
    Returns nearest point to 'target' point, robot can reach
    """
    l1_length = np.linalg.norm(links[:,0])
    l2_length = np.linalg.norm(links[:,1])
    target_length = np.linalg.norm(target-robot_offset)
    inner_radius = np.abs(l1_length-l2_length)
    outer_radius = l1_length + l2_length
    
    if target_length <= outer_radius and target_length >= inner_radius:
        return target
    dir_vec = normalize(target-robot_offset)
    if target_length > outer_radius:
        return robot_offset + outer_radius * dir_vec
    elif target_length < inner_radius:
        return robot_offset + inner_radius * dir_vec
    else:
        raise Exception("Shouldn't happen")

def Test_CorrectPointToRobotEnvelope2D():
    l_3d = np.array([[5,0],
                 [0,6],
                 [0,0]])
    res1 = CorrectPointToRobotEnvelope2D(l_3d,np.array([5,20,0]),np.array([5,5,0]))
    assert (res1 == np.array([5,16,0])).all(),"Assert failed!"
    res2 = CorrectPointToRobotEnvelope2D(l_3d,np.array([-0.5,0,0]),np.array([0,0,0]))
    assert (res2 == np.array([-1,0,0])).all(),"Assert failed!"
    

def IK_With_2_Joints(links,w,target,teta1,teta2):
    offset,new_links, new_w = BuildNew2JointsRobot(links,w,teta1,teta2)
    new_target = CorrectPointToRobotEnvelope2D(new_links,target,offset)
    #[[t11,t12],[t21,t22]] = IK_2_ClosedFormula(new_links,new_w,new_target)
    res = IK_2_ClosedFormula(new_links,new_w,new_target)
    return res
   
def Test_IK_With_2_Joints():
    r_links = np.array([[5,5,5,5],
                        [0,0,0,0],
                        [0,0,0,0]])
    
    w=np.array([[0,0,0,0],
                [0,0,0,0],
                [1,1,1,1]])
    
    target = np.array([20,0,0])
    teta1 = 2
    teta2 = 3
    res = IK_With_2_Joints(r_links,w,target,teta1,teta2)
    #print (np.rad2deg(res))
    #print ("res",res)
    #print (res[0][0])
    tetas = np.array([0.0,0.0,0.0,0.0],)
    tetas[teta1-1] = res[0][0]
    tetas[teta2-1] = res[0][1]    
    #print ("tetas:", tetas)
    t = FK_4(r_links,w,tetas)
    print (t)

#TestBuildNew2JointsRobot()
#Test_CorrectPointToRobotEnvelope2D()
Test_IK_With_2_Joints()
    
    
    
    
    
    
    
    
    
    
    
    
    