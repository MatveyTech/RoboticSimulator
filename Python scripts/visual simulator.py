import pygame, sys
from pygame.locals import *
import numpy as np
from Kinematics import FK_ALL
from Kinematics import FK
from ThesisMinimizer import ThesisMinimizer

# set up the colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (240,240,240)
SLATEGREY = (112,128,144)
WHITESMOKE = (245,245,245)
JOINTSCOLOR = (32,178,170)

squareSize = 1500
screenW = squareSize
screenH = squareSize
scale = 1
ToScreenTransformation = np.array([[scale,0,screenW/2],[0,-scale,screenH/2],[0,0,1]])
FromScreenTransformation = np.linalg.inv(ToScreenTransformation)

def TransformToScreen(point):        
        p = np.array([point[0],point[1],1])
        res = np.dot(ToScreenTransformation,p)
        return (int(res[0]),int(res[1]))
        
    
def TransformFromScreen(point):
    p = np.array([point[0],point[1],1])
    res = np.dot(FromScreenTransformation,p)
    return (int(res[0]),int(res[1]))



class DraggableCircle:
    def __init__(self, color, pos,radius,screenSize):
        self.color = color
        self.position = pos
        self.radius = radius
        self.highlightColor = RED
        self.screenSize = screenSize
        
        
    def Move(self,newPosition):
        self.position = TransformFromScreen(newPosition)
    
    def IsMouseOnObject(self,currenMousePosition):
        mx = TransformFromScreen(currenMousePosition)[0]
        my = TransformFromScreen(currenMousePosition)[1]
        r = self.radius
        px = self.position[0]
        py = self.position[1]
        col = None
        if mx > (px - r) and mx < (px + r) and my > (py - r) and my < (py + r):
            return True
        else:
            return False    
    
    
    def Draw(self,ws,currenMousePosition):
        col = RED if self.IsMouseOnObject(currenMousePosition) else self.color        
        pygame.draw.circle(ws, col, TransformToScreen(self.position), self.radius, 0)
        
class Robot:
#    def __init__(self, links, w, tetas):
#        self.links = links
#        self.j_axes = w
#        self.tetas = tetas
        
    def __init__(self):
        
        self.links = np.array([[200, 150, 250],
                       [0,0,0],
                       [0,0,0]])

    
        self.j_axes=np.array([[0,0,0],
                    [0,0,0],
                    [1,1,1]])
    
        rad10 = np.deg2rad(10)
        #self.tetas = np.array([rad10,rad10,rad10],dtype=float)
        self.tetas = np.array([0.4102, 5.7474, 1.005])
        
        
    def Draw(self,ws):
        fk_all = FK_ALL(self.links,self.j_axes,self.tetas)
        fk_ext = [np.array([0,0,0]),*fk_all] 
        ind = 0
        for i in range(len(fk_ext)-1): 
            p_from = TransformToScreen((fk_ext[ind][0],fk_ext[ind][1]))
            p_to   = TransformToScreen((fk_ext[ind+1][0],fk_ext[ind+1][1]))
            pygame.draw.line(ws,SLATEGREY,p_from,p_to,15)
            ind = ind+1
            
        ind = 0
        p_to = None
        for i in range(len(fk_ext)-1): 
            p_from = TransformToScreen((fk_ext[ind][0],fk_ext[ind][1]))
            p_to   = TransformToScreen((fk_ext[ind+1][0],fk_ext[ind+1][1]))
            pygame.draw.circle(ws, BLACK, p_from, 15, 0)
            ind = ind+1
        pygame.draw.circle(ws, BLACK, p_to, 23, 0)   
        
    def Move(self,tetas):
        self.tetas = tetas
        
    def IsCloseToTarget(self,target):
        threshold = 2
        
        fk = FK(self.links,self.j_axes,self.tetas)   
        delta = fk - target
        dist = np.sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2])
        return dist < threshold
        
def DrawGrid(w,rows,surface):
    sizeBtwn = w // rows
    x = 0
    y = 0
    for l in range(rows):
        x = x + sizeBtwn
        y = y + sizeBtwn
        
        pygame.draw.line(surface,BLACK,(x,0),(x,w))
        pygame.draw.line(surface,BLACK,(0,y),(w,y))


def DrawFrame(surface):
    h = screenH / 2 * 0.8
    w = screenW / 2 * 0.8
    pygame.draw.line(surface,RED,TransformToScreen((-w,h)),TransformToScreen((w,h)),4)
    pygame.draw.line(surface,RED,TransformToScreen((w,h)),TransformToScreen((w,-h)),4)
    pygame.draw.line(surface,RED,TransformToScreen((w,-h)),TransformToScreen((-w,-h)),4)
    pygame.draw.line(surface,RED,TransformToScreen((-w,-h)),TransformToScreen((-w,h)),4)


def DrawMouseCoordinates(m,s):
#    text = font.render('(300,400)', True, BLACK, WHITE)       
#    textRect = text.get_rect() 
#    textRect.center = (100,100) 
#    s.blit(text, textRect)
    mt = TransformFromScreen(m) 
    stri = "({0},{1})".format(mt[0],mt[1])
    pygame.display.set_caption(stri)
    
    
# set up pygame
pygame.init()
font = pygame.font.Font('freesansbold.ttf', 32)

# set up the window
screenSize = (screenW, screenH)
windowSurface = pygame.display.set_mode(screenSize, 0, 32)
pygame.display.set_caption('Thesis')

pygame.draw.circle(windowSurface, BLACK, TransformToScreen((0,0)), 7, 5)

ds1_position = (250,450)
ds = DraggableCircle(BLUE,ds1_position,30,screenSize)
draggingObject = None



robot = Robot()
tm = ThesisMinimizer(robot.links,robot.j_axes,robot.tetas,np.array([ds1_position[0],ds1_position[1],0]))


# run the game loop
i = 0
while True:
    i = i+ 1
#    if i==20:
#        breakדקךכץאקא
    windowSurface.fill(WHITESMOKE)
    DrawGrid(screenW,10,windowSurface)
    DrawFrame(windowSurface)
    #pygame.draw.circle(windowSurface, BLACK, TransformToScreen((0,0)), 7, 5)
    mouse = pygame.mouse.get_pos()
    
    DrawMouseCoordinates(mouse,windowSurface) 
     
    if draggingObject == ds:
        ds.Move(mouse)
        tm.UpdateTarget(np.array([ds.position[0],ds.position[1],0]))
    
    ds.Draw(windowSurface,mouse)
    
    if not robot.IsCloseToTarget(tm.tgt):
        print ("Making step")
        tm.MakeStep()
        robot.Move(tm.theta)
    robot.Draw(windowSurface)
    
    pygame.display.update()
    
    for event in pygame.event.get():        
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            
            if ds.IsMouseOnObject(mouse):
                draggingObject = ds
        elif event.type == pygame.MOUSEBUTTONUP:
            draggingObject = None