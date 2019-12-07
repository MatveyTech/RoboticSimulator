# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 15:43:34 2019

@author: Matvey
"""
from math_utils import *
import numpy as np
np.set_printoptions(suppress=True,precision=2)

def FK_2(links,joint_axes,tetas):
    
    w = joint_axes
    
    R0=CreateRotationMatrix(tetas[0],w[:,0])
    R1=CreateRotationMatrix(tetas[1],w[:,1])
    
    #place of the joint2 in world coordinates
    P1=np.dot(R0,links[:,0])
    P2= np.dot(R0,(links[:,0]+np.dot(R1,links[:,1])))
    
    return P2


def FK_4(links,joint_axes,tetas):
    
    w = joint_axes
    
    R0=CreateRotationMatrix(tetas[0],w[:,0])
    R1=CreateRotationMatrix(tetas[1],w[:,1])
    R2=CreateRotationMatrix(tetas[2],w[:,2])
    R3=CreateRotationMatrix(tetas[3],w[:,3])

    #place of the joint2 in world coordinates
    P1=np.dot(R0,links[:,0])
    P2= np.dot(R0,(links[:,0]+np.dot(R1,links[:,1])))
    P3= np.dot(R0,+links[:,0]+np.dot(R1,(links[:,1]+np.dot(R2,links[:,2]))))
    P4= np.dot(R0,+links[:,0]+np.dot(R1,links[:,1]+np.dot(R2,links[:,2]+np.dot(R3,links[:,3]))))
    
    return P4
    

def IK_4_GaussNewthon(links,w,target_point,starting_theta = np.array([0,0,0,0])):
    l = links
    theta = starting_theta
    num_of_iterations = 0
    
    while True:
        num_of_iterations = num_of_iterations + 1
        
        R0=CreateRotationMatrix(theta[0],w[:,0])
        R1=CreateRotationMatrix(theta[1],w[:,1])
        R2=CreateRotationMatrix(theta[2],w[:,2])
        R3=CreateRotationMatrix(theta[3],w[:,3])
        
        J = np.zeros((3, 4))
        J[:,0] = np.cross(w[:,0],np.dot(R0,(l[:,0]+np.dot(R1,(l[:,1]+np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))))
        J[:,1] = np.dot(R0,np.cross(w[:,1], np.dot(R1,l[:,1]+np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))
        J[:,2] = np.dot(R0,np.dot(R1,np.cross(w[:,0],np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))
        J[:,3] = np.dot(R0,np.dot(R1,np.dot(R3,np.cross(w[:,0],np.dot(R3,l[:,3])))))
        
        FK = FK_4(links,w,theta)
       
        p = np.dot(-np.linalg.pinv(J),(FK-target_point))
        norm_p = np.linalg.norm(p)
        if norm_p < 0.0001:
            break
        theta=theta+p;
    return theta,num_of_iterations

def IK_2_ClosedFormula(links,w,target):   
    #handle special case is when target is 0,0,0
    if (target == 0).all():
        ang = GetAngleBetweenVec(links[:,1],-links[:,0],w[:,1])
        return np.array([[0 ,ang % (2*np.pi)],
                         [0 ,ang % (2*np.pi)]])
        
    l1 = np.linalg.norm(links[:,0])
    l2 = np.linalg.norm(links[:,1])
    x,y,z = target
    #alpha = np.arctan(np.abs(x)/np.abs(y))
    j1_zero_vec = normalize(links[:,0])
    j2_zero_vec = normalize(links[:,1])
    #print("Call1")
    alpha = GetAngleBetweenVec(np.array([x,y,0]),j1_zero_vec,w[:,0])
    b = np.sqrt(x*x+y*y)
    c = np.sqrt(l2*l2-z*z)
    beta = Get_C_OppositeAngle(b,l1,c)   
    #print(np.rad2deg(alpha),np.rad2deg(beta))
    
    teta1_1 = (2*np.pi- alpha - beta) 
    R1 = CreateRotationMatrix(teta1_1,w[:,0])
    link1_rotated = np.dot(R1,links[:,0])
    w1_rotated = np.dot(R1,w[:,1])    
    j2_zero_vec_rotated = np.dot(R1,j2_zero_vec)    
    l2_vec = target - link1_rotated 
    teta1_2 = GetAngleBetweenVec(j2_zero_vec_rotated,l2_vec,w1_rotated)
    
    teta2_1 = (2*np.pi- alpha + beta) 
    R1 = CreateRotationMatrix(teta2_1,w[:,0])
    link1_rotated = np.dot(R1,links[:,0])
    w1_rotated = np.dot(R1,w[:,1])    
    j2_zero_vec_rotated = np.dot(R1,j2_zero_vec)    
    l2_vec = target - link1_rotated 
    teta2_2 = GetAngleBetweenVec(j2_zero_vec_rotated,l2_vec,w1_rotated)
            
    return np.array([[teta1_1 % (2*np.pi) ,teta1_2 % (2*np.pi)],
                     [teta2_1 % (2*np.pi) ,teta2_2 % (2*np.pi)]])

def CalcInverseKinematics2J2D(links,target):
    a =np.linalg.norm(links[:,0])
    b =np.linalg.norm(links[:,1])
    c =np.linalg.norm(target)

    angles = GetTriangleAngles(a,b,c)
    t = np.arcsin(target[1]/c)
    teta1 = -(t+angles[1]) 
    teta2 = np.pi - angles[2] 
    return teta1, teta2
    
    
