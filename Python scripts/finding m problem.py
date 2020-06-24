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
from ThesisMinimizer import ThesisMinimizer
from math_utils import isPositiveDefinite, norm_2

def WriteTextToFile(filename, text):
    f = open(filename, 'a')
    f.write(text)
    f.close()

def BuildNew1JointsRobot(links,w,teta):
    offset,new_links, new_w = BuildNew2JointsRobot(links,w,teta,teta)
    return offset,new_links[:,-1], new_w[:,-1]






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
#        _l = np.array([[drand(mil,mal),drand(mil,mal),drand(mil,mal)],
#                       [0,0,0],
#                       [0,0,0]])
        
        _l = np.array([[5, 5, 5],
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
        
        #_tgt = FK(_l,_w,rand_teta_for_target)
        _tgt = np.array([0.5,0.5,0])

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
        _l = np.array([[5, 5, 5],
                       [0,0,0],
                       [0,0,0]])

    
        _w=np.array([[0,0,0],
                    [0,0,0],
                    [1,1,1]])
    
        rad10 = np.deg2rad(20)
        #_st_pos = np.array([rad10,rad10,rad10],dtype=float)
        _st_pos = np.array([-3.1260556476,  2.0357718904, -2.6774722534])
        
        _tgt = np.array([0.5,0.5,0])
    
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
    
def plotValues(newton_p,thesis_p,title="Title"):    
    n = len(newton_p)
    #plt.plot(range(n),_p)
    #plt.plot(_gr, range(n))
    import matplotlib.pyplot as plt 
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
#    ax.plot(x, y)
#    plt.close(fig)
  
    # plotting the points  
    ax.plot(range(n),newton_p,color='blue',marker='x',markersize=5,linewidth = 6) 
    ax.plot(range(n),thesis_p,color='red',marker='o',markersize=5, linewidth = 6) 
    
#    fig, axs = plt.subplots(2)
#    fig.suptitle(title)
#    axs[0].plot(range(n),_p,color='red')
#    axs[0].set_title('P')
##    axs[1].plot(range(n),_gr)
##    axs[1].set_title('Gradient')
    plt.savefig('foo77.png')
    plt.show()
    
def saveFileResults(newton_p,thesis_p,l,w,tetas,target,fileName):
    n = len(newton_p)
    
    fig, axs = plt.subplots(2,figsize=(20, 10))
    #fig.suptitle("title")
    axs[0].plot(range(n),newton_p,color='blue',marker='x',markersize=5,linewidth = 6, label="Newton") 
    axs[0].plot(range(n),thesis_p,color='red',marker='o',markersize=5, linewidth = 6, label="Thesis") 
    axs[0].set_title('Objective values')
    axs[0].legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
#    axs[1].plot(range(n),range(n))
#    axs[1].set_title('Robot initial state')
    
    frame_xs = [20,-20,-20,20]
    frame_ys = [20,20,-20,-20]
    
    xs = [0]
    ys = [0]       
    
    p_list = FK_ALL(l,w,tetas)
    for p in p_list:
        xs.append(p[0])
        ys.append(p[1])
        
    target_xs = target[0]
    target_ys = target[1]
    
    #print(xs,ys)
#    plt.axis([-20, 20, -20, 20])
#    plt.xticks(np.arange(0, 21, 2)) 
#    plt.yticks(np.arange(0, 11, 2)) 
    
    #self.ax.clear()
    axs[1].plot(xs,ys,marker='o',markersize=9)
    axs[1].plot(frame_xs,frame_ys,color='black', linestyle='dashed', linewidth=0)
    axs[1].plot(target_xs,target_ys,marker='o',markersize=15,color='orange')
    axs[1].annotate('Target', xy=(target[0], target[1]), xytext=(0,20), arrowprops=dict(facecolor='black', shrink=0.05))
    plt.savefig(fileName)
    #plt.show()

def NewPlotFunc(newton_p):
    n = len(newton_p)
    import matplotlib.pyplot as plt
    plt.plot(range(n),newton_p)
    plt.ylabel('some numbers')
    plt.show()


    

############################### I N P U T S #############################
randomValues=False
useVisualization = False
max_numOfIterations = 500
useNewthonMethod=False
################################ I N P U T S #############################
pos_defs = []
not_pos_defs = []

num_of_tests = 1
for test_ind in range(0,num_of_tests):
    print ('Test Number:{0}'.format(test_ind))
    links,w,st_pos,tgt = GetInitialValues3(random=randomValues)
    
    if randomValues:
        print ("Random calculated links:\n",links)
        print ("Random calculated links")
        print ("Random calculated starting position:\n",st_pos)
        print ("Random calculated target:\n",tgt)
    else:
        print ("NOT Random calculated links:\n",links)
        print ("NOT Random calculated starting position:\n",st_pos)
        print ("NOT Random calculated target:\n",tgt)

    fk = FK(links,w,st_pos)    
    #tgt = fk + np.array([0,0.01,0])
    
    theta_newton = np.copy(st_pos)
    theta_thesis = np.copy(st_pos)    
    
    num_of_iterations = 0
    
    if useVisualization:        
        v =Vis(links,w)
        v.DrawRobot(theta_newton,theta_thesis,num_of_iterations,tgt,True)
    
    objective_values_newton = []
    objective_values_thesis = []
    
    newton_reached = False
    thesis_reached = False


    
    goodOutputFileName = 'c:/temp/thesis test/test{0}.png'.format(test_ind)
    badOutputFileName = 'c:/temp/thesis test/test{0}.txt'.format(test_ind)
    pos_def = 0
    npos_def = 0
    while True:        
        num_of_iterations = num_of_iterations + 1    
        print ("-------------------Iteration:",num_of_iterations)         
        
           
        J_newton = CalcJacobian(links,w,theta_newton)   
        forwardK_newton = FK(links,w,theta_newton)

        
        J_thesis = CalcJacobian(links,w,theta_thesis)   
        forwardK_thesis = FK(links,w,theta_thesis)
    
        #A,b,M = CalcAandB(links,w,tgt,theta_thesis)
        tm = ThesisMinimizer(links,w,theta_thesis,tgt)
        #tm.MakeStep()
        A,M = tm.CalcAandBImproved(links,w,tgt,theta_thesis)
#        print(links)
#        print(A)
#        input()
#        is_pos_def = isPositiveDefinite(A)
#        if is_pos_def:
#            pos_def = pos_def + 1
#            print ("    POS DEF!")
#            #WriteTextToFile('c:/temp/thesis test/test{0}_pos_def.txt'.format(test_ind),str(num_of_iterations))
#        else:
#            npos_def = npos_def + 1
#            print ("    NOT POS DEF! Min eig:",np.min(np.linalg.eigvals(A)))
#            #WriteTextToFile('c:/temp/thesis test/test{0}_NOT_pos_def.txt'.format(test_ind),str(num_of_iterations))
#            #print("   eigvals:",np.linalg.eigvals(A)) 
    
        gr_newton = np.dot(np.transpose(J_newton),(forwardK_newton-tgt))
        gr_thesis = np.dot(np.transpose(J_thesis),(forwardK_thesis-tgt))
        
        p_newton = np.dot(-np.linalg.pinv(J_newton),(forwardK_newton-tgt))
        p_newton_threshold = np.linalg.norm(p_newton)
        p_thesis = -np.dot(np.linalg.pinv(A),gr_thesis)
        p_thesis_threshold = np.linalg.norm(p_thesis)
        
        stop_treshold = 0.01
        
#        objective_values_newton.append(np.linalg.norm(forwardK_newton-tgt,2))
#        objective_values_thesis.append(np.linalg.norm(forwardK_thesis-tgt,2))
        
        objective_values_newton.append(norm_2(forwardK_newton-tgt)*0.5)
        objective_values_thesis.append(norm_2(forwardK_thesis-tgt)*0.5)
        
        if  not newton_reached and p_newton_threshold < stop_treshold:
            print("Newton reached the stop threshold")
#            print("Newton threshold",p_newton_threshold)
#            print("Thesis threshold",p_thesis_threshold)
            print ("Iteration #", num_of_iterations)
#            print ("Target position:",tgt)
#            print ("Current position:",FK(links,w,theta_newton))
#            print ("Current theta:",np.rad2deg(theta_newton))
#            print("---------------------------------\n")
            newton_reached = True
            #break
        
        if  not thesis_reached and p_thesis_threshold < stop_treshold:
            print("Thesis reached the stop threshold")
#            print("Newton threshold",p_newton_threshold)
#            print("Thesis threshold",p_thesis_threshold)
#            
            print ("Iteration #", num_of_iterations)
#            print ("Target position:",tgt)
#            print ("Current position:",FK(links,w,theta_thesis))
#            print ("Current theta:",np.rad2deg(theta_thesis))
#            print("\n")
            thesis_reached = True
            #break
        if newton_reached and thesis_reached:            
            saveFileResults(objective_values_newton,objective_values_thesis,links,w,st_pos,tgt,goodOutputFileName)
            break
        
        if num_of_iterations  >= max_numOfIterations:
            error="We reached max num of iterations"
            print (error)       
            WriteTextToFile('c:/temp/thesis test/test{0}_bad_max_num_of_iter.txt'.format(test_ind),error)
            break
        
        if useVisualization:
            v.DrawRobot(theta_newton,theta_thesis,num_of_iterations, tgt,True)
        
        step_newton = None
        step_thesis = None
        try:
            step_newton = tm.doLineSearch(theta_newton,p_newton,links,w,tgt)
            step_thesis = tm.doLineSearch(theta_thesis,p_thesis,links,w,tgt)
        except:
            WriteTextToFile('c:/temp/thesis test/test{0}_line_search_failed.txt'.format(test_ind),"line_search_failed")
            break
        
#        print("Step N",step_newton)
#        print("P N",p_newton)
        #break
        
        #theta_thesis=tm.MakeStep()        
        theta_thesis=(theta_thesis+p_thesis*step_thesis) % (2*np.pi)     
        theta_newton=(theta_newton+p_newton*step_newton) % (2*np.pi)
        
    
    #print("Newton value:",CalcObjectiveValue(theta_newton,tgt,links,w))
    pos_defs.append(pos_def)
    not_pos_defs.append(npos_def)
print ("Done. Number of iterations: ", num_of_iterations)
#plt.clf
#plt.bar(range(num_of_tests), pos_defs,label='Pos def')
#plt.bar(range(num_of_tests), not_pos_defs,label='Not pos def')
#plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
#plt.show()

#plotValues(objective_values_newton,objective_values_thesis)
#if useVisualization:

#NewPlotFunc(objective_values_newton)

