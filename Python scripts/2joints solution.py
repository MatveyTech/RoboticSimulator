#print("Geeks : % 2d, Portal : % 5.2f" %(1, 05.333))  
#%%
import numpy as np
from numpy.linalg import norm
#from Kinematics import FK_2
#from .kinematics import CreateRotationMatrix

verySmallFloat = 0.000001
smallFloat = 0.001
comparisonFloat = 0.05

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


def CalcInverseKinematics2J2D(links,target):
    a =np.linalg.norm(links[:,0])
    b =np.linalg.norm(links[:,1])
    c =np.linalg.norm(target)

    angles = GetTriangleAngles(a,b,c)
    t = np.arcsin(target[1]/c)
    teta1 = -(t+angles[1]) 
    teta2 = np.pi - angles[2] 
    return teta1, teta2

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
    if c < -1 and np.abs(-1 - c) < verySmallFloat:
        c = -1
    if c > 1 and np.abs(1 - c) < verySmallFloat:
        c = 1
    #print("C : % 5.2f" % c)  
    teta = np.arccos(c)
    R=CreateRotationMatrix(teta,cr)
    cand1 = np.dot(R,v1)
    prod1 = np.dot(cand1,v2)
    if np.abs(prod1 - 1) < verySmallFloat:
        return teta
    else:
        teta = 2*np.pi-teta
        R=CreateRotationMatrix(teta,cr)
        cand2 = np.dot(R,v1)
        prod2 = np.dot(cand2,v2)
        if np.abs(prod2 - 1) < smallFloat:
            return teta
    raise ValueError('The vec angle function failed')
    
    
def CalcInverseKinematics2J3D(links,w,target):    
    
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

l_3d = np.array([[5,0],
                 [0,6],
                 [0,0]])

w = np.array([[0,1],
              [0,0],
              [1,0]])

theta = np.array([np.deg2rad(0),np.deg2rad(0)])
#point_3d = FK_2(l_3d,w,theta)
point_3d = [5,6,10]
print(point_3d)
[[t11,t12],[t21,t22]] = CalcInverseKinematics2J3D(l_3d,w,point_3d)
target_for_verification1 = FK_2(l_3d,w,np.array([t11,t12]))
target_for_verification2 = FK_2(l_3d,w,np.array([t21,t22]))
print(np.rad2deg([t11,t12]),target_for_verification1)
print(np.rad2deg([t21,t22]),target_for_verification2)
print (compare(target_for_verification1,target_for_verification2))
#%%
ran = 360
for t_1 in range(ran):
    for t_2 in range(ran): 
        print(t_1,t_2)
        theta = np.array([np.deg2rad(t_1),np.deg2rad(t_2)])
        point_3d = FK_2(l_3d,w,theta)
        [[t11,t12],[t21,t22]] = CalcInverseKinematics2J3D(l_3d,w,point_3d)
        target_for_verification1 = FK_2(l_3d,w,np.array([t11,t12]))
        target_for_verification2 = FK_2(l_3d,w,np.array([t21,t22]))
        res1 = compare(point_3d,target_for_verification1)
        res2 = compare(point_3d,target_for_verification2)
        input()
        if not res1:
            print("first pair failed",t_1,t_2,point_3d,target_for_verification1,res)
            input()
        if not res2:
            print("second pair failed",t_1,t_2,point_3d,target_for_verification2,res)
            input()
    print(t_1)
        
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

