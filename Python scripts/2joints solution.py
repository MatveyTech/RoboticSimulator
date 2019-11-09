#print("Geeks : % 2d, Portal : % 5.2f" %(1, 05.333))  
#%%
import math_utils
import Kinematics
import numpy as np
from numpy.linalg import norm

def CorrectPointToRobotEnvelope2D(links,target):
    """    
    Returns nearest point to 'target' point, robot can reach
    """
    l1_length = np.linalg.norm(links[:,0])
    l2_length = np.linalg.norm(links[:,1])
    target_length = np.linalg.norm(target)
    inner_radius = np.abs(l1_length-l2_length)
    outer_radius = l1_length + l2_length
    
    if target_length <= outer_radius and target_length >= inner_radius:
        return target
    dir_vec = normalize(target)
    if target_length > outer_radius:
        return outer_radius * dir_vec
    elif target_length < inner_radius:
        return inner_radius * dir_vec
    else:
        raise Exception("Shouldn't happen")
        
    

l_3d = np.array([[5,0],
                 [0,6],
                 [0,0]])

w = np.array([[0,1],
              [0,0],
              [1,0]])

theta = np.array([np.deg2rad(0),np.deg2rad(0)])
#point_3d = FK_2(l_3d,w,theta)
point_3d = [5,6,0]
print(point_3d)
[[t11,t12],[t21,t22]] = CalcInverseKinematics2J3D(l_3d,w,point_3d)
target_for_verification1 = FK_2(l_3d,w,np.array([t11,t12]))
target_for_verification2 = FK_2(l_3d,w,np.array([t21,t22]))
print(np.rad2deg([t11,t12]),target_for_verification1)
print(np.rad2deg([t21,t22]),target_for_verification2)
print (compare(target_for_verification1,target_for_verification2))
#%% New semester 2020A. First meeting with Roi
print (CorrectPointToRobotEnvelope2D(l_3d,np.array([-0.5,0,0])))
#%%
n = np.array([2,2,0])
x = np.array([6,0,0]) + 3 * n
print (x)
#%%
import matplotlib.pyplot as plt
l1 = []
l2 = []
r = 90
d = 360/r
rot_mat = CreateRotationMatrix(np.deg2rad(d),np.array([0,0,1]))
p = np.array([3,0,0])
for step_1 in range(r):
    p = np.dot(rot_mat,p)
    l1.append(p[0])
    l2.append(p[1])
    #print (p)

plt.figure(figsize=(10,10),dpi=100, facecolor='w', edgecolor='red')
plt.scatter(l1,l2)
plt.show()


#%% Testing all permutations of thetas
ran = 360
for t_1 in range(ran):
    for t_2 in range(ran): 
        #print(t_1,t_2)
        theta = np.array([np.deg2rad(t_1),np.deg2rad(t_2)])
        point_3d = FK_2(l_3d,w,theta)
        [[t11,t12],[t21,t22]] = CalcInverseKinematics2J3D(l_3d,w,point_3d)
        target_for_verification1 = FK_2(l_3d,w,np.array([t11,t12]))
        target_for_verification2 = FK_2(l_3d,w,np.array([t21,t22]))
        res1 = compare(point_3d,target_for_verification1)
        res2 = compare(point_3d,target_for_verification2)
        #input()
        if not res1:
            print("first pair failed",t_1,t_2,point_3d,target_for_verification1,res)
            input()
        if not res2:
            print("second pair failed",t_1,t_2,point_3d,target_for_verification2,res)
            input()
    #print(t_1)
        
#%%
# 2D
l_2d = np.array([[5,5],[0,0]])
point_2d = np.array([5,5])
t1,t2 = CalcInverseKinematics2J2D(l_2d,point_2d)
print (np.rad2deg(t1),np.rad2deg(t2))

#%%  Test GetAngleBetweenVec function    
x = np.array([1,1,0])
y = np.array([1,0,0])
z = np.array([0,0,1])
print(np.rad2deg(GetAngleBetweenVec(y,x,z)))
#%%
t1 = np.deg2rad(259.6111421845303)
#t1 = np.deg2rad(0)
t2 = np.deg2rad(0)

l = np.array([[5,0],
                 [0,6],
                 [0,0]])

w = np.array([[0,1],
              [0,0],
              [1,0]])


#t1 = np.deg2rad(270)
#t2 = np.deg2rad(0)
FK_2(l,w,np.array([t1,t2]))

#%%
l_3d = np.array([[5,0],
                 [0,6],
                 [0,0]])

w = np.array([[0,1],
              [0,0],
              [1,0]])

t4 = np.array([np.deg2rad(100.38885782),np.deg2rad(180)])
p = FK_2(l_3d,w,t4)
print(p)

#%% Plot example
import numpy as np
import matplotlib.pyplot as plt
np.set_printoptions(precision=1,suppress=True)

l_3d_test = np.array([[5,20],
                      [0,0],
                      [0,0]])

w_test = np.array([[0,0],
                   [0,0],
                   [1,1]])

l1 = []
l2 = []

ran = 360
step = 9
for t_1 in range(0,ran,step):
    for t_2 in range(0,ran,step): 
        #print(t_1,t_2)
        theta = np.array([np.deg2rad(t_1),np.deg2rad(t_2)])
        #input(theta)
        point_3d = FK_2(l_3d_test,w_test,theta)
        l1.append(point_3d[0])
        l2.append(point_3d[1])

plt.figure(figsize=(10,10),dpi=100, facecolor='w', edgecolor='red')
plt.scatter(l1,l2)
plt.show()
