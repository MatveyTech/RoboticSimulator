# -*- coding: utf-8 -*-
"""
Created on Thu Apr  9 22:23:11 2020

@author: Matvey Rzhavskiy
"""
from Kinematics import FK
from Kinematics import CalcJacobian
from math_utils import norm_2
from Types import Variables
import numpy as np





class CompitabilityObj:
           
        
    def GetCloserT(self,pair,current):
        dist1 = np.linalg.norm(pair[0]-current)
        dist2 = np.linalg.norm(pair[1]-current)
        if dist1 <= dist2:
            return pair[0]  
        else:
            return pair[1]  
    
    def BuildNew2JointsRobot(self,links,w,teta1,teta2,curr_pos):
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
    
    
    def CorrectPointToRobotEnvelope2D_2J(self,links,target,robot_offset = np.array([0,0,0])):
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
    
    def IK_With_2_Joints(self,links,w,target,teta1,teta2,curr_pos):
        offset,new_links, new_w = self.BuildNew2JointsRobot(links,w,teta1,teta2,curr_pos)
        new_target_orig_space = self.CorrectPointToRobotEnvelope2D_2J(new_links,target,offset)
        new_target_new_space = new_target_orig_space - offset
        #[[t11,t12],[t21,t22]] = IK_2_ClosedFormula(new_links,new_w,new_target)
        res = IK_2_ClosedFormula(new_links,new_w,new_target_new_space)
        return res
    
    def CalcObjectiveValue(self,thetas,tgt,links,w):
        fk = FK(links,w,thetas)
        t = fk - tgt
        res = t[0]*t[0] + t[1]*t[1] + t[2]*t[2]
        return res
    
    
    def doLineSearch(self,thetas,descent,links,w,target):
        #print ("descent",descent)
        lineSearchStartValue = 1
        maxNumOfIterations = 15
        
        alpha = lineSearchStartValue
        initialObjValue = self.CalcObjectiveValue(thetas,target,links,w)
        
        for i in range(0,maxNumOfIterations):
            new_thetas = (thetas + descent * alpha) % (2*np.pi)
            newObjValue = self.CalcObjectiveValue(new_thetas,target,links,w)
            if newObjValue < initialObjValue:
                return alpha
            else:
                alpha = alpha / 2
        
        raise("line search failed")
    
    def SingleIteration(self,curr_pos,links,w,target,i,j):
        if i > i:
            raise "the first index shouldn't begreater than the second"
        if i == j:
            res = IK_With_1_Joint(links,w,target,i)
            x = np.copy(curr_pos)
            x[i-1] = res
        else: 
            res = self.IK_With_2_Joints(links,w,target,i,j,curr_pos)
            closer = self.GetCloserT(res,np.array([curr_pos[i-1],curr_pos[j-1]]))
            x = np.copy(curr_pos)
            x[i-1] = closer[0]
            x[j-1] = closer[1] 
        return x
    
    def Build_Single_X_Matrix(self,x_i):
        s = x_i.size
        res = np.zeros((s,s*s))
        for i in range(s):
            res[i,i*s:i*s+s] = x_i
        return res
    
    def Build_Big_M_Matrix_and_bb(self,links,w,target,curr_pos):       
        n = links.shape[1]
        res = np.empty([0, n*n])
        b = -calculateGradient(links,w,target,curr_pos)
        res_bb = []
        for i in range(1,n+1):
            for j in range(i+1,n+1):
                #print ("\n\nCurrent tetas : ",i,j)
                sing_x = self.SingleIteration(curr_pos,links,w,target,i,j)
                #
                #print("x_ij:",i,j,np.rad2deg(sing_x))
                p_ij = sing_x - curr_pos
                #print("p_ij:",i,j,np.rad2deg(p_ij))
                #sing_x = SingleIteration(starting_position,links,w,target,3,4)
                mat = self.Build_Single_X_Matrix(p_ij)
                improved_mat = mat[[i-1,j-1], :]
                #print (mat)
    #            print ("res shape: ",res.shape)
    #            print ("mat shape: ",mat.shape)
                
                res = np.row_stack((res,improved_mat))
                res_bb.append(b[i-1])
                res_bb.append(b[j-1])
        return res,np.asarray(res_bb)
    
    def calculate_A_symmetric3(self,mat_M,vec_b):
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
        #m2 = np.linalg.inv(m1)
        m2 = np.linalg.pinv(m1)
        m3 = np.dot(m2,mat_MS_T)
        As = np.dot(m3,vec_b)    
        
        res_A = np.array([As[0],As[1],As[2],As[1],As[3],As[4],As[2],As[4],As[5]])    
        
        new_shape_size = int(np.sqrt(res_A.shape[0]))
        return res_A.reshape(new_shape_size,new_shape_size)  
    
    def CalcAandBImproved(self,p_links,p_w,p_tgt,p_curr_pos):
        M,bb = self.Build_Big_M_Matrix_and_bb(p_links,p_w,p_tgt,p_curr_pos)   
        #res_a = calculate_A(M,bb) 
        res_a = self.calculate_A_symmetric3(M,bb)     
        return res_a,M  
    
    
#   #   #                    P    U    B    L    I    C    S        #####
    
    def __init__(self,nj,npts,links,axes,w=1):
        self.nj = nj
        self.npts = npts
        self.links = links
        self.axes = axes       
        self.weight = w
        self.inner_w = 1e-2
        
        
    def ComputeValue(self,curr):
        val = 0
        for i in range(self.npts):
            theta_i = curr.GetTheta(i)
            ee_i = curr.GetEE(i)
            fk_i = FK(self.links,self.axes, theta_i)
            tres = norm_2(fk_i-ee_i)
            val += tres
        #return val * self.weight * 2
        return self.inner_w * val * self.weight
    
    def AddGradientTo(self,curr,grad):
        tgrad = Variables(curr.nJ,curr.nP)
        for i in range(self.npts):
            theta_i = curr.GetTheta(i)
            ee_i = curr.GetEE(i)
            jac_i = CalcJacobian(self.links,self.axes,theta_i)   
            fk_i = FK(self.links,self.axes, theta_i)  
            #print(theta_i,ee_i,theta_i,fk_i)
            grad_t = np.dot(np.transpose(jac_i),(fk_i- ee_i)) 
            grad_e = (fk_i- ee_i) * (-1)
            tgrad.SetTheta(i,grad_t)
            tgrad.SetEE(i,grad_e)
        grad  += self.inner_w * tgrad.data  * 2  * self.weight
        
        
    def AddHessianTo(self,curr,hess):
        dp = 1e-5
        data = curr.data
        for i in range(curr.shape):
            C_P = np.zeros(data.shape,np.float64)
            C_M = np.zeros(data.shape,np.float64)
            tmpVal = data[i]
            data[i] = tmpVal + dp
            self.AddGradientTo(curr,C_P)
            data[i] = tmpVal - dp
            self.AddGradientTo(curr,C_M)  
            data[i] = tmpVal
            res = (C_P - C_M)/(2*dp)
            hess[i] += res
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    