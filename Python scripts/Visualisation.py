# -*- coding: utf-8 -*-
"""
Created on Fri Dec 20 13:02:57 2019

@author: rzhavm2
"""

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
from Kinematics import FK_ALL

class Vis:
    def __init__(self,l,w):
        style.use('fivethirtyeight')
        fig = plt.figure()
        #plt.ylim(0, 20)
        
        self.ax = fig.add_subplot(1,1,1)
        self.ax.set_autoscale_on(False)
        #plt.axes().set_aspect('equal')
        #plt.axis([-20, 20, -20, 20],equal=True)
        plt.subplot(111)
        #plt.plot(3*np.cos(an), 3*np.sin(an))
        plt.axis('equal')
        plt.axis([-10, 10, -20, 20])
        plt.xticks(np.arange(0, 21, 3)) 
        plt.yticks(np.arange(0, 11, 3)) 
        self.links = l
        self.j_axes = w


    def DrawRobot(self,tetas, iter_num,target=np.array([0,0,0]),wait_for_click_to_cont = False):
        p_list = FK_ALL(self.links,self.j_axes,tetas)
        frame_xs = [20,-20,-20,20]
        frame_ys = [20,20,-20,-20]
        
        xs = [0]
        ys = [0]
        
        
        
        for p in p_list:
            xs.append(p[0])
            ys.append(p[1])
            
        target_xs = target[0]
        target_ys = target[1]
        
        #print(xs,ys)
        plt.axis([-20, 20, -20, 20])
        plt.xticks(np.arange(0, 21, 2)) 
        plt.yticks(np.arange(0, 11, 2)) 
        
        self.ax.clear()
        self.ax.plot(xs,ys,marker='o',markersize=9)
        self.ax.plot(frame_xs,frame_ys,color='black', linestyle='dashed', linewidth=0)
        #self.ax.plot(target_xs,target_ys,'rX',linewidth=5)
        self.ax.plot(target_xs,target_ys,marker='o',markersize=9)
        #self.ax.plot(joints_xs,joints_xy, marker='o',markersize=6, color='green')
        
        #self.ax.text(0.95, 0.01, 'colored text in axes coords', verticalalignment='bottom', horizontalalignment='left',transform=self.ax.transAxes,color='green', fontsize=15)
        #self.ax.plot([2], [1], 'o')
        self.ax.annotate('Target', xy=(target[0], target[1]), xytext=(0,20), arrowprops=dict(facecolor='black', shrink=0.05))
        iterationString = 'Iteration:{0}'.format(iter_num)
        self.ax.text(-20, 25, iterationString)
        if wait_for_click_to_cont:
            x = plt.ginput()
