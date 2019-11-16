#print("Geeks : % 2d, Portal : % 5.2f" %(1, 05.333))  
#%%
from math_utils import *
from Kinematics import *
import numpy as np
from numpy.linalg import norm


        
    

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
[[t11,t12],[t21,t22]] = IK_2_ClosedFormula(l_3d,w,point_3d)
target_for_verification1 = FK_2(l_3d,w,np.array([t11,t12]))
target_for_verification2 = FK_2(l_3d,w,np.array([t21,t22]))
print(np.rad2deg([t11,t12]),target_for_verification1)
print(np.rad2deg([t21,t22]),target_for_verification2)
print (compare(target_for_verification1,target_for_verification2))
#%% New semester 2020A. First meeting with Roi
print (CorrectPointToRobotEnvelope2D(l_3d,np.array([-0.5,0,0])))

#%% Testing all permutations of thetas
ran = 360
for t_1 in range(ran):
    for t_2 in range(ran): 
        #print(t_1,t_2)
        theta = np.array([np.deg2rad(t_1),np.deg2rad(t_2)])
        point_3d = FK_2(l_3d,w,theta)
        [[t11,t12],[t21,t22]] = IK_2_ClosedFormula(l_3d,w,point_3d)
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
        

