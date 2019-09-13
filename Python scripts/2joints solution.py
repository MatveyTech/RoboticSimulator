
#%%
import numpy as np

def Get_C_OppositeAngle(a,b,c):
    #return np.rad2deg(np.arccos((a*a+b*b-c*c)/(2*a*b)))
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

def CalcInverseKinematics2J3D(links,target):
    l1 = np.linalg.norm(links[:,0])
    l2 = np.linalg.norm(links[:,1])
    x,y,z = target
    alpha = np.arctan(x/y)
    b = np.sqrt(x*x+y*y)
    c = np.sqrt(l2*l2-z*z)
    beta = Get_C_OppositeAngle(b,l1,c)
    
    print(np.rad2deg(alpha),np.rad2deg(beta))
    teta1 = np.pi /2 - alpha - beta
    print (teta1)

l_3d = np.array([[5,0],[0,6],[0,0]])
point_3d = np.array([5,4.24, 4.24])
CalcInverseKinematics2J3D(l_3d,point_3d)
#%%
# 2D
l_2d = np.array([[5,5],[0,0]])
point_2d = np.array([5,5])
t1,t2 = CalcInverseKinematics2J2D(l_2d,point_2d)
print (np.rad2deg(t1),np.rad2deg(t2))



