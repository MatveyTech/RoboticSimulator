# -*- coding: utf-8 -*-
"""
Created on Sat Nov 16 11:30:00 2019

@author: rzhavm2
"""
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

#%%
import numpy as np
import matplotlib.pyplot as plt
plt.figure(figsize=(15,15),dpi=100, facecolor='w', edgecolor='red')
x, y = np.random.random(size=(2,10))

for i in range(0, len(x), 2):
    plt.plot(x[i:i+2], y[i:i+2], 'ro-')

plt.show()
#%%
import matplotlib.pyplot as plt

x=[-1 ,0.5 ,1,-0.5]
y=[ 0.5,  1, -0.5, -1]

plt.plot(x,y, 'ro')

def connectpoints(x,y,p1,p2):
    x1, x2 = x[p1], x[p2]
    y1, y2 = y[p1], y[p2]
    plt.plot([x1,x2],[y1,y2],'k-')

connectpoints(x,y,0,1)
connectpoints(x,y,2,3)

plt.axis('equal')
plt.show()

#%%
def draw_line():
    plt.figure(figsize=(8,8),dpi=150, facecolor='w', edgecolor='red')
    # List to hold x values.
    x_number_values = [1, 10]
    # List to hold y values.
    y_number_values = [1, 10]    
    # Plot the number in the list and set the line thickness.
   
    # Set the x, y axis tick marks text size.
    plt.tick_params(axis='both', labelsize=9)
    # Display the plot in the matplotlib's viewer.
    plt.show()

draw_line()