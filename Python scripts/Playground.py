# -*- coding: utf-8 -*-
"""
Created on Tue Apr 14 17:15:07 2020

@author: rzhavm2
"""


from Minimizers import GradientDescentFunctionMinimizer
from Minimizers import NewtonFunctionMinimizer
from Objectives import TheObjective
from Objectives import StartObj
from Objectives import FinalObj
from Objectives import SmoothnessObj
from IKObjective import IK_Obj
import numpy as np
from Types import Variables
from Kinematics import FK
from math_utils import norm_2, scalar_prod,normalize,isPositiveDefinite
from math_utils import drand, testSparsityPattern, AddEpsToDiagonal
import sys
from math_utils import getMinEigenValue

#class XSquare:
#        
#    def ComputeValue(self,p):
#        x = p.data[0]
#        if x < 0:
#            return x * x
#        else:
#            return x * x * x
#    
#    def AddGradientTo(self,p,grad):
#        x = p.data[0]
#        if x < 0:
#            tgrad = 2 * x
#        else:
#            tgrad = 3 * x * x       
#        grad[0]  += tgrad  
#
#x = Variables(1,1)
#x.data[0] = 1000000000000000000000000000000
##x = np.array([10])
#
#m = GradientDescentFunctionMinimizer(1)
#obj = XSquare()
#c = False
#p = x
#while c is False:
#    p,c = m.minimize(obj,p)
#    print (p.data,c)    
    
li = np.array([[200, 150, 250],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([100,0,0],dtype='f')
end_point = np.array([-100,0,0],dtype='f')

li = np.array([[20, 15, 25],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([10,0,0],dtype='f')
end_point = np.array([-10,0,0],dtype='f')

li = np.array([[2, 1.5, 2.5],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([1,0,0],dtype='f')
end_point = np.array([-1,0,0],dtype='f')

#just for debugging, remove this later
li = np.array([[5,5,5],[0,0,0],[0,0,0]],dtype='f')
ax = np.array([[0,0,0],[0,0,0],[1,1,1]])
st_point = np.array([1,0,0],dtype='f')
end_point = np.array([-1,0,0],dtype='f')

m = GradientDescentFunctionMinimizer(1)
m = NewtonFunctionMinimizer()

numT = 3
numP = 5


x = drand(-np.pi,np.pi)
y = drand(-np.pi,np.pi)
z = drand(-np.pi,np.pi)


#obj = SmoothnessObj(0,4)

v = Variables(numT,numP)
for i in range(numP):
    v.SetEE(i,np.array([0.5,0.5,0],dtype='f'))
    #v.SetTheta(i,np.array([3,3,3]))
    v.SetTheta(i,np.array([drand(-np.pi,np.pi),drand(-np.pi,np.pi),drand(-np.pi,np.pi)]))
#v.SetData(np.array([5.0485032785, 0.6399702503, 5.2151616457,0.5,0.5,0]))
#v.SetData(np.array([x,y,z,0.5,0.5,0]))
#v.SetData(np.array([-1.4486789355, -1.9841015962, -1.8320902301,0.5,0.5,0]))

print ("Initial guess",v)
st_point = np.array([0.5,0.5,0],dtype='f')
objN = TheObjective(st_point,end_point,numT,numP,li,ax,False,np.array([100,100,10,100],dtype='f'))
objT = TheObjective(st_point,end_point,numT,numP,li,ax,True,np.array([100,100,10,100],dtype='f'))
#v = tt
#v = np.array([100,0,0])


#tgt = FK(li,ax,v)

#tgt = np.array([0.5,0.5,0])
#objN = IK_Obj(li,ax,tgt)
#objT = IK_Obj(li,ax,tgt,useA=True)
#m = NewtonFunctionMinimizer(isVarValue=False)
#v = np.array([5.0485032785, 0.6399702503, 5.2151616457])
#v = np.array([x,y,z])


#objN.TestGradientWithFD(v)
#for i in range(100):
#    mil = 0
#    mal = 10.1
#    #v = np.array([drand(mil,mal),drand(mil,mal),drand(mil,mal)])
#    objN.TestGradientWithFD(v)
#sys.exit()
#    mil = 3.0
#        mal = 9.99
#        _l = np.array([[drand(mil,mal),drand(mil,mal),drand(mil,mal)],


cT = cN = False
pN = v
pT = v
#p = ps[180]
pTs = []
pNs = []

pTvals = []
pNvals = []
T_PDfs = []
N_PDfs = []

sdNs = []
sdTs = []

T_min_eig_vals = []
N_min_eig_vals = []
    
T_grad_sizes = []
N_grad_sizes = []
    
T_numOf_regs = []
N_numOf_regs = []

counter = 0
numOfIterations = 20
runIterationsAnyway = True

print("Initial value:",objN.ComputeValue(pN))

while (runIterationsAnyway and counter<numOfIterations) or (cT is False and cN is False and counter<numOfIterations):
    counter +=1
    print ("- - - - - - - - - - - - - - - - -")
    print ("Iteration:",counter)
    print ("- - - - - - - - - - - - - - - - -")
    
    objN.TestGradientWithFD(pN)
    #objN.TestHessianWithFD(p)
    
    nVal = objN.ComputeValue(pN)
    tVal = objT.ComputeValue(pT)
#    print("pN",pN)
#    print ("Nval:",nVal)
#    print ("Tval:",tVal)
    
    NGrad = objN.CalculateGradient(pN)
    TGrad = objT.CalculateGradient(pT)    
    
    
    NSearchDir = m.computeSearchDirection(objN,pN,regFactor=0)
    TSearchDir = m.computeSearchDirection(objT,pT,regFactor=0)
    
    NisDirectionDescent = m.isSearchDirectionDescent(NSearchDir,NGrad)
    TisDirectionDescent = m.isSearchDirectionDescent(TSearchDir,TGrad)
    
#    print(np.linalg.eigvals(objT.CalculateHessian(pT)))
#    print(scalar_prod(normalize(TSearchDir),normalize(TGrad)))
    
    #regularization now
    TnumOfReg = 0
    Tfactor = 0
    Tfactor = 10e-5
    while not TisDirectionDescent:  
        TnumOfReg = TnumOfReg + 1
        Tfactor = Tfactor * 10
        TSearchDir = m.computeSearchDirection(objT,pT,regFactor=Tfactor)
        TisDirectionDescent = m.isSearchDirectionDescent(TSearchDir,TGrad)
        
    #print ("T reg num::", TnumOfReg)   
    
    
    NnumOfReg = 0
    Nfactor = 0
    Nfactor = 10e-5
    while not NisDirectionDescent:  
        NnumOfReg = NnumOfReg + 1
        Nfactor = Nfactor * 10
        NSearchDir = m.computeSearchDirection(objN,pN,regFactor=Nfactor)
        NisDirectionDescent = m.isSearchDirectionDescent(NSearchDir,NGrad)
        
    #print ("N reg num::", NnumOfReg)   
    
    
    NHess = objN.CalculateHessian(pN) #this one may be fifferent fro the one used in computeSearchDirection
    THess = objT.CalculateHessian(pT) #this one may be fifferent fro the one used in computeSearchDirection
    
    AddEpsToDiagonal(THess,Tfactor)
    AddEpsToDiagonal(NHess,Nfactor)
    
    #print (np.linalg.eigvals(NHess))
    
    #print("N: descentDirection? ",NisDirectionDescent)
    #print("T: descentDirection? ",TisDirectionDescent)
    
    
    NStepSize = norm_2(NSearchDir)
    TStepSize = norm_2(TSearchDir)
    
    #print("Step N",step_newton)
    #print("P N",NSearchDir)
    
    #break
    
    
    
    
#    isSparsityOk = testSparsityPattern(NHess,THess)
#    print ("IsSparsityOK: ",isSparsityOk)
    
#    print("Obj N value:",nVal)
#    print("pN:",pN)
    #print("FK(pN):",FK(li,ax,pN))
#    print("Obj T value:",tVal)
#    print("N Step size:",NStepSize)
#    print("T Step size:",TStepSize)
#    
#    
#    print("N: p * grad :",scalar_prod(normalize(NSearchDir),normalize(NGrad)))
#    print("N Hessian is positive definite? ", isPositiveDefinite(NHess))
#    if not isPositiveDefinite(NHess):
#        print("   eigvals:",np.linalg.eigvals(NHess))
#    
#    print("T: p * grad :",scalar_prod(normalize(TSearchDir),normalize(TGrad)))
#    print("T Hessian is positive definite? ", isPositiveDefinite(THess))
#    if not isPositiveDefinite(THess):
#        print("   eigvals:",np.linalg.eigvals(THess))
    #print("minimizing N")
    
    T_min_eig_vals.append(getMinEigenValue(objT.CalculateHessian(pT)))
    N_min_eig_vals.append(getMinEigenValue(objN.CalculateHessian(pN)))
    
    pN,cN = m.minimize(objN,pN,Nfactor)  
    #print("minimizing T")
    pT,cT = m.minimize(objT,pT,Tfactor)  
    
    pTs.append(pT)
    pNs.append(pN)
    pNvals.append(nVal)
    pTvals.append(tVal)
    sdNs.append(NStepSize)
    sdTs.append(TStepSize)
    
    
    
    T_grad_sizes.append(norm_2(TGrad))
    N_grad_sizes.append(norm_2(NGrad))
    
    T_numOf_regs.append(TnumOfReg)
    N_numOf_regs.append(NnumOfReg)
    
    
    T_PDfs.append(not isPositiveDefinite(THess))    
    N_PDfs.append(not isPositiveDefinite(NHess)) 

#    t = p.GetTheta(i)
#    e = p.GetEE(i)
#    print (FK(li,ax,t)-e)

#print("Newton")
#print("  FK:",norm_2(FK(li,ax,pN.GetTheta(0))-pN.GetEE(0)))
#print("  FK:",norm_2(st_point-pN.GetEE(0)))
#
#print("Thesis")
#print("  FK:",norm_2(FK(li,ax,pT.GetTheta(0))-pT.GetEE(0)))
#print("  FK:",norm_2(st_point-pT.GetEE(0)))
    
#print(T_PDfs)    
#print(N_PDfs)
    #print (p,c) 
plot = True
if plot:
    import matplotlib.pyplot as plt
    # %matplotlib qt
    
    plt.figure(figsize=(9,9))
    
    plt.subplot(221)
    #X = range(len(pNvals))
    X = np.arange(len(pNvals))
    plt.plot(X, pNvals,label='Newton')
    plt.plot(X, pTvals,label='Thesis')
    plt.title("Obj values")
    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
    
    
    
    #
#    plt.subplot(222)
#    plt.plot(X, sdNs,label='Newton')
#    plt.plot(X, sdTs,label='Thesis')
#    plt.title("Step size")
#    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
    
    plt.subplot(222)
    plt.plot(X, N_grad_sizes,label='Newton',)
    plt.plot(X, T_grad_sizes,label='Thesis')
    plt.title("Gradient Size")
    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
    
    plt.subplot(223)
    plt.plot(X, N_min_eig_vals,label='Newton')
    plt.plot(X, T_min_eig_vals,label='Thesis')
    plt.title("Min eigen value")
    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
    
    plt.subplot(224)
    plt.plot(X, N_numOf_regs,label='Newton')
    plt.plot(X, T_numOf_regs,label='Thesis')
    plt.title("Num of regs")
    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
    #ii = 7
    
    
    
#    T_min_eig_vals = []
#N_min_eig_vals = []
#    
#T_grad_sizes = []
#N_grad_sizes = []
#    
#T_numOf_regs = []
#T_numOf_regs = []
    
#    plt.subplot(313)
#    plt.bar(X,N_PDfs, width = 0.25,color = 'b')
#    plt.bar(X+0.25,T_PDfs, width = 0.25,color = 'orange')
#    plt.legend(labels=['Newton', 'Thesis'])
    
#    plt.subplot(413)
#    plt.plot(X, sdNs,label='Newton')
#    plt.plot(X, sdTs,label='Thesis')
#    plt.title("Step size")
#    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)
#    
#    plt.subplot(414)
#    plt.plot(X, sdNs,label='Newton')
#    plt.plot(X, sdTs,label='Thesis')
#    plt.title("Step size")
#    plt.legend(bbox_to_anchor=(0.5, 0.95), loc='upper left', borderaxespad=0.)  
#    ii = ii+1
#    plt.savefig("C:/temp/4/{0}.png".format(ii))
    plt.show()    
#    
print("\n")
#print (p,c)
#res_str = print("\n\033[1;31;47mMinimization method: {0}. Num of iterations: {1}.".format(m.GetName(),counter))

print("\n\033[1;31;47mMinimization method: {0}. Num of iterations: {1}. Converged: {2}".format(m.GetName(),counter,cN))
print("\n\033[1;31;47mMinimization method: {0}. Num of iterations: {1}. Converged: {2}".format("Thesis minimizer",counter,cT))


#for i in range(p.nP):
#    t = p.GetTheta(i)
#    e = p.GetEE(i)
#    print (FK(li,ax,t)-e)

#fk = FK(li,ax, np.array([1.1440029882e+03,  7.8483575105e+02,  5.2133984492e+02]))
#print (fk)  
