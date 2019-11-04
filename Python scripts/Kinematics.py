# -*- coding: utf-8 -*-
"""
Created on Sat Sep  7 15:43:34 2019

@author: rzhavm2
"""

#%%
import numpy as np

np.set_printoptions(suppress=True,precision=2)

def CreateRotationMatrix(teta,u):
    """    
    Returns rotation matrix to rotate teta readians around u
    """
    [x,y,z] = u
    c = np.cos(teta)
    s = np.sin(teta)
    res = np.array([[c+x*x*(1-c),x*y*(1-c)-z*s,x*z*(1-c)+y*s],[y*x*(1-c)+z*s,c+y*y*(1-c),y*z*(1-c)-x*s],[z*x*(1-c)-y*s,z*y*(1-c)+x*s,c+z*z*(1-c)]]) 
    return res

def FK_2(links,joint_axes,tetas):
    
    w = joint_axes
    
    R0=CreateRotationMatrix(tetas[0],w[:,0])
    R1=CreateRotationMatrix(tetas[1],w[:,1])
    
    #place of the joint2 in world coordinates
    P1=np.dot(R0,links[:,0])
    P2= np.dot(R0,(links[:,0]+np.dot(R1,links[:,1])))
    
#    print(np.rad2deg(tetas[0]),np.rad2deg(tetas[1]),P2)
#    input()
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
    

def IK_4_Newthon(links,w,target_point,starting_theta = np.array([0,0,0,0])):
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
        
        #print("J : ",J)
        
        FK = FK_4(links,w,theta)
        #print(FK)
        #print ("FK : ", FK)         
        
        p = np.dot(-np.linalg.pinv(J),(FK-target_point))        
        #print (p)
        norm_p = np.linalg.norm(p)
        print ("norm p :", norm_p) 
        if norm_p < 0.0001:
            break
        print ("Iteration #", num_of_iterations)
        theta=theta+p;
    return theta,num_of_iterations
    
#%%
#joint locations in local coordinates. Root is always at [0,0,0]


#this one is input for FK


#%%
r_links = np.array([[5,5,5,5],
                    [0,0,0,0],
                    [0,0,0,0]])

w=np.array([[0,0,0,0],
            [0,0,0,0],
            [1,1,1,1]])

target = np.array([20,20,0])

res_theta,num_of_it = IK_4_Newthon(r_links,w,target)
    
print ("\nNumber of iterations :", num_of_it) 
print (np.rad2deg(res_theta)%360)
print ("FK : ", FK_4(r_links,w,res_theta))
#%%


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