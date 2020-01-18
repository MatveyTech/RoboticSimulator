# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 15:43:34 2019

@author: Matvey
"""
from math_utils import *
import numpy as np
np.set_printoptions(suppress=False,precision=20)
np.set_printoptions(linewidth=160)

def FK_2_ALL(links,joint_axes,tetas):
    
    w = joint_axes
    
    R0=CreateRotationMatrix(tetas[0],w[:,0])
    R1=CreateRotationMatrix(tetas[1],w[:,1])
    
    #place of the joint2 in world coordinates
    P1=np.dot(R0,links[:,0])
    P2= np.dot(R0,(links[:,0]+np.dot(R1,links[:,1])))
    
    return [P1,P2]

def FK_2(links,joint_axes,tetas):
    p_list = FK_2_ALL(links,joint_axes,tetas)    
    return p_list[1]
    

def FK_3_ALL(links,joint_axes,tetas):
    
    w = joint_axes
    
    R0=CreateRotationMatrix(tetas[0],w[:,0])
    R1=CreateRotationMatrix(tetas[1],w[:,1])
    R2=CreateRotationMatrix(tetas[2],w[:,2])

    #place of the joint2 in world coordinates
    P1=np.dot(R0,links[:,0])
    P2= np.dot(R0,(links[:,0]+np.dot(R1,links[:,1])))
    P3= np.dot(R0,+links[:,0]+np.dot(R1,(links[:,1]+np.dot(R2,links[:,2]))))
    
    return [P1,P2,P3]

def FK_3(links,joint_axes,tetas):
    p_list = FK_3_ALL(links,joint_axes,tetas)    
    return p_list[2]
    

def FK_4_ALL(links,joint_axes,tetas):
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
    return [P1,P2,P3,P4]

def FK_4(links,joint_axes,tetas):
    p_list = FK_ALL(links,joint_axes,tetas)    
    return p_list[3]

def FK(links,joint_axes,tetas):
    cl = links.shape[1]
    cj = joint_axes.shape[1]
    ct = tetas.shape[0]
    if cl != cj or cj != ct:
        print ("links size:",cl)
        print ("axes  size:",cj)
        print ("tetas size:",ct)
        raise "FK: Bad dimentions!!!"
    if cl == 2:
        return FK_2(links,joint_axes,tetas)
    elif cl == 3:
        return FK_3(links,joint_axes,tetas)
    elif cl == 4:
        return FK_4(links,joint_axes,tetas)
    else:
        raise "FK: Current implementation works only on 2,3 or 4 links!"
        
def FK_ALL(links,joint_axes,tetas):
    cl = links.shape[1]
    cj = joint_axes.shape[1]
    ct = tetas.shape[0]
    if cl != cj or cj != ct:
        raise "FK_ALL: Bad dimentions!!!"
    if cl == 2:
        return FK_2_ALL(links,joint_axes,tetas)
    elif cl == 3:
        return FK_3_ALL(links,joint_axes,tetas)
    elif cl == 4:
        return FK_4_ALL(links,joint_axes,tetas)
    else:
        raise "FK: Current implementation works only on 2,3 or 4 links!"


def CalcJacobian(l,w,theta):
    size = l.shape[1]
    if size == 4:
        return CalcJacobian4(l,w,theta)
    elif size == 2:
        return CalcJacobian2(l,w,theta)
    elif size == 3:
        return CalcJacobian3(l,w,theta)
    else:
        raise "Jacobian not of size 2, 3 or 4"
        
    

def CalcJacobian2(l,w,theta):
    
    R0=CreateRotationMatrix(theta[0],w[:,0])
    R1=CreateRotationMatrix(theta[1],w[:,1])
        
    J = np.zeros((3, 2))
    J[:,0] = np.cross(w[:,0],np.dot(R0,(l[:,0]+np.dot(R1,(l[:,1])))))
    J[:,1] = np.dot(R0,np.cross(w[:,1], np.dot(R1,l[:,1])))
        
    return J

def CalcJacobian3(l,w,theta):
    
    R0=CreateRotationMatrix(theta[0],w[:,0])
    R1=CreateRotationMatrix(theta[1],w[:,1])
    R2=CreateRotationMatrix(theta[2],w[:,2])    
    
    J = np.zeros((3, 3))
    J[:,0] = np.cross(w[:,0],np.dot(R0,(l[:,0]+np.dot(R1,(l[:,1]+np.dot(R2,l[:,2]))))))
    J[:,1] = np.dot(R0,np.cross(w[:,1], np.dot(R1,l[:,1]+np.dot(R2,l[:,2]))))
    J[:,2] = np.dot(R0,np.dot(R1,np.cross(w[:,0],np.dot(R2,l[:,2]))))
        
    return J

def CalcJacobian4(l,w,theta):
    
    R0=CreateRotationMatrix(theta[0],w[:,0])
    R1=CreateRotationMatrix(theta[1],w[:,1])
    R2=CreateRotationMatrix(theta[2],w[:,2])
    R3=CreateRotationMatrix(theta[3],w[:,3])
    
    J = np.zeros((3, 4))
    J[:,0] = np.cross(w[:,0],np.dot(R0,(l[:,0]+np.dot(R1,(l[:,1]+np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))))
    J[:,1] = np.dot(R0,np.cross(w[:,1], np.dot(R1,l[:,1]+np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))
    J[:,2] = np.dot(R0,np.dot(R1,np.cross(w[:,0],np.dot(R2,l[:,2]+np.dot(R3,l[:,3])))))
    J[:,3] = np.dot(R0,np.dot(R1,np.dot(R3,np.cross(w[:,0],np.dot(R3,l[:,3])))))
    
    return J
    

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
        
        J = CalcJacobian(links,w,theta)
        
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
    
    
