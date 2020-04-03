import pygame, sys
from pygame.locals import *
import numpy as np
from Kinematics import FK_ALL
from Kinematics import FK
from ThesisMinimizer import ThesisMinimizer
from Types import Variables
from Objectives import CompitabilityObj 


# set up the colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (240,240,240)
SLATEGREY = (112,128,144)
HALF_SLATEGREY = (184, 192, 200)
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
        
    def __init__(self,isSimulation=False):
        
        self.links = np.array([[200, 150, 250],
                       [0,0,0],
                       [0,0,0]])

    
        self.j_axes=np.array([[0,0,0],
                    [0,0,0],
                    [1,1,1]])
    
        rad10 = np.deg2rad(10)
        #self.tetas = np.array([rad10,rad10,rad10],dtype=float)
        self.tetas = np.array([0.4102, 5.7474, 1.005])
        self.linksColor = SLATEGREY if not isSimulation else HALF_SLATEGREY
        self.linksWidth = 15 if not isSimulation else 8
        self.jointsRad =  15 if not isSimulation else 8
        self.eeRad =  23 if not isSimulation else 14
        
    def Draw(self,ws):
        fk_all = FK_ALL(self.links,self.j_axes,self.tetas)
        fk_ext = [np.array([0,0,0]),*fk_all] 
        ind = 0
        for i in range(len(fk_ext)-1): 
            p_from = TransformToScreen((fk_ext[ind][0],fk_ext[ind][1]))
            p_to   = TransformToScreen((fk_ext[ind+1][0],fk_ext[ind+1][1]))
            pygame.draw.line(ws,self.linksColor,p_from,p_to,self.linksWidth)
            ind = ind+1
            
        ind = 0
        p_to = None
        for i in range(len(fk_ext)-1): 
            p_from = TransformToScreen((fk_ext[ind][0],fk_ext[ind][1]))
            p_to   = TransformToScreen((fk_ext[ind+1][0],fk_ext[ind+1][1]))
            pygame.draw.circle(ws, BLACK, p_from, self.jointsRad, 0)
            ind = ind+1
        pygame.draw.circle(ws, BLACK, p_to, self.eeRad, 0)   
        
    def Move(self,tetas):
        self.tetas = tetas
        
    def IsCloseToTarget(self,target):
        threshold = 1
        
        fk = FK(self.links,self.j_axes,self.tetas)   
        delta = fk - target
        dist = np.sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2])
        return dist < threshold


        
        
class Simulation:
    def __init__(self,npts):
        self.nJ = 3
        self.nP = npts
        self.Robots = np.ndarray((npts,),dtype=np.object)
        #self.var = Variables(self.nJ,self.nP)

        for i in range(npts):
            self.Robots[i] = Robot(True)
    
    
    def Move(self,var):
        v = Variables(self.nJ,self.nP,var)
        for i in range(self.nP):
            self.Robots[i].Move(v.GetTheta(i))
        
    
    def Draw(self,windowSurface):
        for i in range(self.nP):
            self.Robots[i].Draw(windowSurface)
        
#sim = Simulation(10)



#y = Variables(2,2)
#print (y.GetTheta(0))
#y.SetEE(0,np.array([100,200,300]))
#print (y.GetTheta(0))
#print (y.data)
#
#print (y.GetEE(1))
        
        
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
    

def EqualPath(start,end,nP):
    jn = start.shape[0]    
    blockSize = jn + 3
    step_arr = (end-start) / nP
    res  = np.zeros(nP*(jn+3))
    res[0:jn] = start
    lastBlockInd = blockSize*(nP - 1)
    res[lastBlockInd:lastBlockInd+jn] = end
    for i in range(1,nP-1): 
        res[i*blockSize:i*blockSize+jn] = start + i * step_arr
    return res
    
 
# set up pygame
pygame.init()
font = pygame.font.Font('freesansbold.ttf', 32)

# set up the window
screenSize = (screenW, screenH)
windowSurface = pygame.display.set_mode(screenSize, 0, 32)
pygame.display.set_caption('Thesis')

tgt_st = DraggableCircle(BLUE,( 445,145),30,screenSize)
tgt_en   = DraggableCircle(BLUE,(-250,450),30,screenSize)

draggingObject = None

State = AppState.Rest

robot_start = Robot()
robot_final = Robot()
num_Of_Points = 10
sim = Simulation(num_Of_Points)
#tm = ThesisMinimizer(robot.links,robot.j_axes,robot.tetas,np.array([ds1_position[0],ds1_position[1],0]))
target = np.array([600,0,0])
tm_start = ThesisMinimizer(robot_start.links,robot_start.j_axes,robot_start.tetas,target)
tm_final = ThesisMinimizer(robot_final.links,robot_final.j_axes,robot_final.tetas,-target)

IsCalculatingPathNow = False

# run the game loop
while True:
    windowSurface.fill(WHITESMOKE)
    DrawGrid(screenW,10,windowSurface)
    DrawFrame(windowSurface)
    mouse = pygame.mouse.get_pos()
    
    DrawMouseCoordinates(mouse,windowSurface)     
    
        
    if draggingObject == tgt_st:
        draggingObject.Move(mouse)
        tm_start.UpdateTarget(np.array([draggingObject.position[0],draggingObject.position[1],0]))
        
    if draggingObject == tgt_en:
        draggingObject.Move(mouse)
        tm_final.UpdateTarget(np.array([draggingObject.position[0],draggingObject.position[1],0]))
    
    tgt_st.Draw(windowSurface,mouse)
    tgt_en.Draw(windowSurface,mouse)
    
    if  IsCalculatingPathNow:
        v = EqualPath(robot_start.tetas,robot_final.tetas,num_Of_Points)
        sim.Move(v)
        #print (np.rad2deg(v))
        #break
        sim.Draw(windowSurface)
    
    if not robot_start.IsCloseToTarget(tm_start.tgt):
        print ("Making step start")
        tm_start.MakeStep()
        robot_start.Move(tm_start.theta)
    robot_start.Draw(windowSurface)
    
    if not robot_final.IsCloseToTarget(tm_final.tgt):
        print ("Making final start")
        tm_final.MakeStep()
        robot_final.Move(tm_final.theta)
    robot_final.Draw(windowSurface)
    
    

        
    pygame.display.update()
    
#    b = ReadBoolFromIni("Autostep")
#    print(b)
    
    for event in pygame.event.get():        
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            
            if tgt_st.IsMouseOnObject(mouse):
                draggingObject = tgt_st
            elif tgt_en.IsMouseOnObject(mouse):
                draggingObject = tgt_en
        elif event.type == pygame.MOUSEBUTTONUP:
            draggingObject = None
        elif event.type == pygame.KEYDOWN:
            IsCalculatingPathNow = not IsCalculatingPathNow
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            