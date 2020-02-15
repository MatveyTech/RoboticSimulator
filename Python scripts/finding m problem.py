# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 08:56:50 2019

@author: rzhavm2
"""

import numpy as np
import random
#import Kinematics
from Kinematics import *
from Visualisation import Vis
import matplotlib.pyplot as plt

def BuildNew1JointsRobot(links,w,teta):
    offset,new_links, new_w = BuildNew2JointsRobot(links,w,teta,teta)
    return offset,new_links[:,-1], new_w[:,-1]

def BuildNew2JointsRobotOld(links,w,teta1,teta2):
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
    new_w = np.column_stack(([w[:,teta1-1],w[:,teta2-1]]))
    return offset, np.column_stack((end_of_first_link,end_of_second_link)), new_w


def BuildNew2JointsRobot(links,w,teta1,teta2,curr_pos):
    """    
    This function works only for 2D now!!!!!
    teta1 and teta2 are starting from 1..
    """
    ii = teta1-1
    jj = teta2-1
    
    temp_curr_pos = np.copy(curr_pos)
    
    temp_curr_pos[ii]=0
    temp_curr_pos[jj]=0
    
    fkall = FK_ALL(links,w,temp_curr_pos)
    fk_ext = [np.array([0,0,0]),*fkall]    
    
    offset = fk_ext[ii]
    end_of_first_link = fk_ext[jj] - fk_ext[ii]
    end_of_second_link = fk_ext[-1]  - fk_ext[jj]
       
    new_w = np.column_stack(([w[:,ii],w[:,jj]]))
    return offset, np.column_stack((end_of_first_link,end_of_second_link)), new_w

def TestBuildNew2JointsRobot():
    r_links = np.array([[5,5,5,5],
                        [0,0,0,0],
                        [0,0,0,0]])
    
    w=np.array([[0,0,0,0],
                [0,0,0,0],
                [1,1,1,1]])
    
    target = np.array([0,20,0])
    
    offset,new_links, new_w = BuildNew2JointsRobot(r_links,w,4,4)
    
    print ("Old links :") 
    print (r_links)
    print ("New links :") 
    print (new_links)
    print ("Offset :") 
    print (offset)
    print ("W : :") 
    print (new_w)

def CorrectPointToRobotEnvelope2D_2J(links,target,robot_offset = np.array([0,0,0])):
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

def CorrectPointToRobotEnvelope2D_1J(link,target,robot_offset = np.array([0,0,0])):
    target_length = np.linalg.norm(target-robot_offset)
    dir_vec = normalize(target-robot_offset)
    radius = np.linalg.norm(link)
    return robot_offset + radius * dir_vec
    

def Test_CorrectPointToRobotEnvelope2D_2J():
    l_3d = np.array([[5,0],
                 [0,6],
                 [0,0]])
    res1 = CorrectPointToRobotEnvelope2D_2J(l_3d,np.array([5,20,0]),np.array([5,5,0]))
    assert (res1 == np.array([5,16,0])).all(),"Assert failed!"
    res2 = CorrectPointToRobotEnvelope2D_2J(l_3d,np.array([-0.5,0,0]),np.array([0,0,0]))
    assert (res2 == np.array([-1,0,0])).all(),"Assert failed!"
    

def IK_With_2_Joints(links,w,target,teta1,teta2,curr_pos):
    offset,new_links, new_w = BuildNew2JointsRobot(links,w,teta1,teta2,curr_pos)
    new_target_orig_space = CorrectPointToRobotEnvelope2D_2J(new_links,target,offset)
    new_target_new_space = new_target_orig_space - offset
    #[[t11,t12],[t21,t22]] = IK_2_ClosedFormula(new_links,new_w,new_target)
    res = IK_2_ClosedFormula(new_links,new_w,new_target_new_space)
    return res

def IK_With_1_Joint(links,w,target,joint_number):
    """
    This function works only in 2D for now!!!!! 
    """
    offset,new_link, new_w = BuildNew1JointsRobot(links,w,joint_number)
    new_target = CorrectPointToRobotEnvelope2D_1J(new_link,target,offset)
    new_target_corrected = new_target - offset
    return np.arctan2(new_target_corrected[1],new_target_corrected[0])
        
   
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
    

def Test_IK_With_1_Joint():
    r_links = np.array([[5,5,5,5],
                        [0,0,0,0],
                        [0,0,0,0]])
    
    w=np.array([[0,0,0,0],
                [0,0,0,0],
                [1,1,1,1]])
    
    target = np.array([-20,20,0])
    teta = 2
    res = IK_With_1_Joint(r_links,w,target,teta)
    tetas = np.array([0.0,0.0,0.0,0.0])
#    print ("res :", res)
    
#    print ("teta :", teta)
    tetas[teta-1] = res
    print ("tetas :", np.rad2deg(tetas))
    t = FK(r_links,w,tetas)
    print ("FK :", t)


def Build_Single_X_Matrix(x_i):
    s = x_i.size
    res = np.zeros((s,s*s))
    for i in range(s):
        res[i,i*s:i*s+s] = x_i
    return res

def GetCloserT(pair,current):
    dist1 = np.linalg.norm(pair[0]-current)
    dist2 = np.linalg.norm(pair[1]-current)
    if dist1 <= dist2:
        return pair[0]  
    else:
        return pair[1]  

def SingleIteration(curr_pos,links,w,target,i,j):
    if i > i:
        raise "the first index shouldn't begreater than the second"
    if i == j:
        res = IK_With_1_Joint(links,w,target,i)
        x = np.copy(curr_pos)
        x[i-1] = res
    else: 
        res = IK_With_2_Joints(links,w,target,i,j,curr_pos)
        
        
        
        closer = GetCloserT(res,np.array([curr_pos[i-1],curr_pos[j-1]]))
        #print ("i: %d, j: %d, closer %d" %(i,j, closer))
        #print("i,j,closer ", i,j,closer)
        
        
        x = np.copy(curr_pos)
        x[i-1] = closer[0]
        x[j-1] = closer[1] 
        
#        y = np.zeros(st_pos.shape)
#        y[i-1] = closer[0]
#        y[j-1] = closer[1] 
#        
#        t_fk = FK(links,w,y)
#        t_fk_norm = np.linalg.norm(t_fk) 
#        print("t_fk(y)", t_fk,y)

        #print(res)
        
    return x
    
    

def Build_Big_M_Matrix(links,w,target,curr_pos):   
    
    n = links.shape[1]
    res = np.empty([0, n*n])
    for i in range(1,n+1):
        for j in range(i+1,n+1):
            #print ("\n\nCurrent tetas : ",i,j)
            sing_x = SingleIteration(curr_pos,links,w,target,i,j)
            #
            #print("x_ij:",i,j,np.rad2deg(sing_x))
            p_ij = sing_x - curr_pos
            #print("p_ij:",i,j,np.rad2deg(p_ij))
            #sing_x = SingleIteration(starting_position,links,w,target,3,4)
            mat = Build_Single_X_Matrix(p_ij)
            #print (mat)
#            print ("res shape: ",res.shape)
#            print ("mat shape: ",mat.shape)
            
            res = np.row_stack((res,mat))
#            if counter==5:
#                break
    return res

def calculateGradient(links,w,target,curr_pos):
    
    J = CalcJacobian(links,w,curr_pos)
    J = np.transpose(J)
    _fk = FK(links,w,curr_pos)
    grad = np.dot(J,_fk-target)
    return grad

def calculate_A(mat_M,vec_b):
    mat_M_T = np.transpose(mat_M)
    m1 = np.dot(mat_M_T,mat_M)
    temp = 1.0001  * np.identity(m1.shape[0])
    m2 = np.linalg.inv(m1+temp)
    m3 = np.dot(m2,mat_M_T)
    res = np.dot(m3,vec_b)
    
    #res = np.dot(np.linalg.pinv(mat_M),vec_b)
    new_shape_size = int(np.sqrt(res.shape[0]))
    return res.reshape(new_shape_size,new_shape_size)   


def calculate_A_symmetric3(mat_M,vec_b):
    S = np.array([[1,0,0,0,0,0],
                  [0,1,0,0,0,0],
                  [0,0,1,0,0,0],
                  [0,1,0,0,0,0],
                  [0,0,0,1,0,0],
                  [0,0,0,0,1,0],
                  [0,0,1,0,0,0],
                  [0,0,0,0,1,0],
                  [0,0,0,0,0,1]])
    
    mat_MS = np.dot(mat_M,S)    
    mat_MS_T = np.transpose(mat_MS)
    m1 = np.dot(mat_MS_T,mat_MS)
    temp = 1.0001  * np.identity(m1.shape[0])
    m2 = np.linalg.inv(m1)
    m3 = np.dot(m2,mat_MS_T)
    As = np.dot(m3,vec_b)
    
    #res = np.dot(np.linalg.pinv(mat_M),vec_b)
    
    
    res_A = np.array([As[0],As[1],As[2],As[1],As[3],As[4],As[2],As[4],As[5]])
    
    
    new_shape_size = int(np.sqrt(res_A.shape[0]))
    return res_A.reshape(new_shape_size,new_shape_size)   

def test_A(links,w,target,starting_position,_b,_A):   
    
    n = links.shape[1]
    res = np.empty([0, n*n])
    for i in range(1,n+1):
        for j in range(i+1,n+1):
            x_ij = SingleIteration(starting_position,links,w,target,i,j)
            p_ij = x_ij - starting_position
            curr_res = -_b + np.dot(_A,p_ij)
            print("",curr_res)
            input("Wait here")
    

def CalcAandB(p_links,p_w,p_tgt,p_curr_pos):
    M = Build_Big_M_Matrix(p_links,p_w,p_tgt,p_curr_pos)
    
    #M2 = 1.0001  * np.identity(9) #works only foe 3 links!!!!!
    #M = M + M2
    
#    print ("The M Matrix:")
#    print(M)
    
#    print ("The T(T(M)) Matrix:")
#    print(np.transpose(np.transpose(M)))
    
#    print ("The pinv(M) Matrix:")
#    print (np.linalg.pinv(M))

#    eig_val_M, eig_vec_M = np.linalg.eig(M)
#    print ("The M eigen val:",eig_val_M)
    
    
    #input (M.shape)
    b = -calculateGradient(p_links,p_w,p_tgt,p_curr_pos)
    bb_required_size = int(M.shape[0] / b.shape[0])
    bb = np.tile(b, bb_required_size)
    #res_a = calculate_A(M,bb) 
    res_a = calculate_A_symmetric3(M,bb)     
    return res_a,b,M        

def drand(min_value,max_value):
    return random.uniform(min_value,max_value)

def GetInitialValues3(random):
    if random:
        mil = 3.0
        mal = 9.99
        _l = np.array([[drand(mil,mal),drand(mil,mal),drand(mil,mal)],
                       [0,0,0],
                       [0,0,0]])
        _w=np.array([[0,0,0],
                    [0,0,0],
                    [1,1,1]])
        
        #_st_pos = np.array([drand(0,2*np.pi),2*np.pi,2*np.pi],dtype=float)
        _st_pos = np.array([drand(0,2*np.pi),drand(0,2*np.pi),drand(0,2*np.pi)],dtype=float)
                
        #for calculating theta let's take any random numbers for thetas
        #and calculate forward kinematics
        
               
        rand_teta_for_target = np.array([drand(0,2*np.pi),drand(0,2*np.pi),drand(0,2*np.pi)],dtype=float)
        #rand_teta_for_target = np.array([drand(0,2*np.pi),0,0],dtype=float)
        
        _tgt = FK(_l,_w,rand_teta_for_target)

    else:
#        _l = np.array([[3.3466,8.9891,8.177],
#                       [0,0,0],
#                       [0,0,0]])
#
#    
#        _w=np.array([[0,0,0],
#                    [0,0,0],
#                    [1,1,1]])
#    
#
#        _st_pos = np.array([3.2003,3.8838,5.2478],dtype=float)
#        
#        _tgt = np.array([6.8304,8.0209,0])
#        
        _l = np.array([[5,5,5],
                       [0,0,0],
                       [0,0,0]])

    
        _w=np.array([[0,0,0],
                    [0,0,0],
                    [1,1,1]])
    
        rad10 = np.deg2rad(20)
        _st_pos = np.array([rad10,rad10,rad10],dtype=float)
        
        _tgt = np.array([0,10,0])
    
    return _l,_w,_st_pos,_tgt

def tweakA(mat_A):
    eig_val_matA, eig_vec_matA = np.linalg.eig(mat_A)
    eig_vec_matA_inverse = np.linalg.inv(eig_vec_matA)
    
    epsilon = 0.1
    D_mat = np.where(eig_val_matA<epsilon, epsilon, eig_val_matA)
    D_mat = np.diag(D_mat)
    
    
    
    
    res = np.dot(np.dot(eig_vec_matA,D_mat),eig_vec_matA_inverse)
    e1,e2 = np.linalg.eig(res)
    print("D_mat:",D_mat)
    print("Eigens inside tweak function",e1)
    #input (res.shape)
    return res

def plotPandGradient(_p,_gr,title="Title"):    
    n = len(_p)
    #plt.plot(range(n),_p)
    #plt.plot(_gr, range(n))
    
    
    fig, axs = plt.subplots(2)
    fig.suptitle(title)
    axs[0].plot(range(n),_p)
    axs[0].set_title('P')
    axs[1].plot(range(n),_gr)
    axs[1].set_title('Gradient')
    
    plt.show()

################################ I N P U T S #############################
randomValues=True
useVisualization = False
max_numOfIterations = 800
useNewthonMethod=False
################################ I N P U T S #############################




links,w,st_pos,tgt = GetInitialValues3(random=randomValues)

if randomValues:
    print ("Random calculated links:\n",links)
    print ("Random calculated starting position:\n",st_pos)
    print ("Random calculated target:\n",tgt)
else:
    print ("NOT Random calculated links:\n",links)
    print ("NOT Random calculated starting position:\n",st_pos)
    print ("NOT Random calculated target:\n",tgt)

fk = FK(links,w,st_pos)

#tgt = fk + np.array([0,0.01,0])
theta = st_pos
num_of_iterations = 0

if useVisualization:
    v =Vis(links,w)
    v.DrawRobot(theta,num_of_iterations,tgt,True)

all_ps = []
all_grads = []

while True:
    
    num_of_iterations = num_of_iterations + 1    
    print ("-------------------Iteration:",num_of_iterations)
    
    
       
    J = CalcJacobian(links,w,theta)   
    forwardK = FK(links,w,theta)
#    print("Current Joints:",theta)
#    temp_x = SingleIteration(theta,links,w,tgt,1,2)
#    print("Suggested joints:",temp_x)
#    print("Target:",tgt)
#    print("FK:",FK(links,w,temp_x))   
##    
##    
##    #temp_gr = np.dot(np.transpose(J),(forwardK-tgt))
#    temp_gr = calculateGradient(links,w,tgt,temp_x)
##        
##    
#    print ("temp_gr",temp_gr)
#    break

    #print ("Current teta:",theta)
    A,b,M = CalcAandB(links,w,tgt,theta)
    #input(M)
    #A = tweakA(A)
    
    #print ("A:\n",A) 
    
    M_T = np.transpose(M)
    M2 = np.dot(M_T,M)
    eig_val_M2, eig_vec_M2 = np.linalg.eig(M2)
    #print ("The MTM eigen val:",eig_val_M2)
    threshold = 0.003
    #print ("Eigen values M2:",eig_val_M2)
    small_eigen_values = (eig_val_M2 < threshold).sum()
    print ("MTM: Eigen values smaller than % 5.3f : %2d" %(threshold, small_eigen_values))
    
    print ("Extreme values(A):",np.amax(A),np.amin(A))
    
    
#    if small_eigen_values > 0:
#         print ("\nS T O P P E D. There are %d values above the threshold(%5.3f) " %(small_eigen_values,threshold))
#         breaktweakA),b)
    
    
#    if useNewthonMethod:
#        H = np.linalgtrans.pinv(J)
#        
#    else:
#        H = np.linalg.pinv(A)
        
    b=forwardK-tgt
    gr = np.dot(np.transpose(J),(forwardK-tgt))
#    p = -np.dot(H,g)
    if useNewthonMethod:
        p = np.dot(-np.linalg.pinv(J),(forwardK-tgt))
    else:
        p = -np.dot(np.linalg.pinv(A),gr)
#    print("gr:",gr)
#    print("b:",b)
#    input()
#    break
    b = gr
    
    norm_p = np.linalg.norm(p) 
    all_ps.append(norm_p)
    
    grad = calculateGradient(links,w,tgt,theta)
    all_grads.append(norm(grad))
    
    print ("\nP: %s, %5.3f\n" %(np.array2string(p), norm_p))
    #print ("Gradient pure, Gradient from A calc ",norm(grad),norm(-b))
    print ("Gradient: %5.3f" %(norm(grad)))
    #input()
    print ("The A matrix:\n",A)
    eig_val, eig_vec = np.linalg.eig(A)    
    print ("A matrix eig_val",eig_val) 
#    all_a_eig_vals.S
    #print ("M matrix\n",M)
       
    
    print ("Target position:",tgt)
    print ("Current position:",FK(links,w,theta))
    
    if norm_p < 0.001:
        print ("\n\n Finished: \n")
        
        print ("(p) ",p)
        print ("norm_p is:", norm_p)
        print ("(Gradient) ",np.linalg.norm(-b))
        
        print ("The A matrix:\n",A)
        eig_val, eig_vec = np.linalg.eig(A)    
        print ("A matrix eig_val",eig_val)        
        
        print ("Target position:",tgt)
        print ("Current position:",FK(links,w,theta))
        break
    
    if num_of_iterations  >= max_numOfIterations:
        print ("We reached max num of iterations")        
        break
    
    if useVisualization:
        v.DrawRobot(theta,num_of_iterations, tgt,True)
    theta=(theta+p*0.05) % (2*np.pi)
    #input ("Enter")

print ("Done. Number of iterations: ", num_of_iterations)

if useVisualization:
    plotPandGradient(all_ps,all_grads,'Newthon' if useNewthonMethod else 'Thesis')
#plotPandGradient(all_grads,all_ps)

#print(A)
#A=np.array([[89.71,0],
#             [0,21.75]])
#A=np.array([[8.91,26.73],
#             [6.52,19.57]])

#pp =np.array([0.52,1.57])
#tt = np.dot(A,pp)
#print("A*(x*-x))= ",tt)    
#    
#A_INV = np.linalg.pinv(A)
#p = np.dot(A_INV,b)
#
#print("p: ",p)

#test_A(links,w,tgt,st_pos,b,A)
##%%
#import numpy as np
##import Kinematics
#from Kinematics import *
#from Visualisation import Vis
#
#
#links = np.array([[5,5,5,5],
#                  [0,0,0,0],
#                  [0,0,0,0]])
#
#    
#w=np.array([[0,0,0,0],
#            [0,0,0,0],
#            [1,1,1,1]])
#
#
#
#v =Vis(links,w)
#theta = np.array([0,0,0,np.deg2rad(90)])
#v.DrawRobot(theta)
#
##%%
#while True:
#    num_of_iterations = num_of_iterations + 1
#       
#    J = CalcJacobian(links,w,theta)   
#    FK = FK_4(links,w,theta)
#   
#    #p = np.dot(-np.linalg.pinv(J),(FK-tgt))
#    p = np.dot(-np.linalg.inv(A),(FK-tgt))
#    
#    norm_p = np.linalg.norm(p)
#    
#    theta=theta+p;
#    v.DrawRobot(theta,tgt,True)
#    if norm_p < 0.0001:
#        break
#
#%%
#test_links = np.array([[5,6],
#                       [0,0],
#                       [0,0]])
#
#test_w=np.array([[0,0],
#                 [0,0],
#                 [1,1]])
#    
#test_target = np.array([0,9,0])
#    
#temp_res = IK_2_ClosedFormula(test_links,test_w,test_target)
#print(temp_res)
#
##%%
#tmp_new_links = np.load('c:/temp/1/new_links.npy')
#tmp_new_w = np.load('c:/temp/1/new_w.npy')
#tmp_new_target_new_space = np.load('c:/temp/1/new_target_new_space.npy')
#
#
#
#tmp_res = IK_2_ClosedFormula(tmp_new_links,tmp_new_w,tmp_new_target_new_space)
#tmp_fk = FK(tmp_new_links,tmp_new_w,tmp_res[0])