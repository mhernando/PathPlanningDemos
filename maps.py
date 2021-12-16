'''this file implements the common map operations'''

import random
import math
import pygame
from Definitions import *


map1=(
(150,80,750,20),
(500,70,20,430),
(50,500,500,20)
)
map2=(
(500,300,50,50),
)
map3=(
(800,450,100,25),
(800,525,100,25),
(800,475,25,50),
(875,475,25,50)
)
maps=(map1,map2, map3)

def p2distance(p1,p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**0.5

def module(v):
    return (v[0]**2+v[1]**2)**0.5

def distance_point_to_segment(p,s1,s2):
    '''returns the distance and the segment point'''
    ab = (s2[0]-s1[0], s2[1]-s1[1])
    ap = (p[0]-s1[0],p[1]-s1[1])
    bp = (p[0]-s2[0],p[1]-s2[1])
    proyection = ab[0]*ap[0]+ab[1]*ap[1]
    m = ab[0]**2+ab[1]**2
    if proyection <= 0:
        return module(ap),s1
    if proyection >= m:
        return module(bp), s2
    pi = (int(s1[0]+proyection*ab[0]/m), int(s1[1]+proyection*ab[1]/m))
    return module((p[0]-pi[0],p[1]-pi[1])), pi

def optimal_radius(n, map_size = 1000, gamma = 0.7):
    return gamma*map_size*(math.log(n+1)/(n+1))**0.5

class BaseMap:
    ''' BaseMap: includes the possibility of drawing it, and the creation of obstacles. 
    It has the methods for collision checking and includes de random generation of C samples'''
    def __init__(self, w,h):
        self._heigh, self._width=h,w
        #window settings
        self._windowName='UPM - Guiado y Navegación de Robots '
        pygame.display.set_caption(self._windowName)
        self.canvas=pygame.display.set_mode((self._width, self._heigh))
        #obstacles
        self._obs=[]
        self._min_obs_size = min_obs_size
        self._obs_variance = obs_variance
  
    def makeRandomRect(self):
        w= int(random.uniform(self._min_obs_size,self._obs_variance))
        h= int(random.uniform(self._min_obs_size,self._obs_variance))
        upx = int(random.uniform(0,self._width-w))
        upy = int(random.uniform(0,self._heigh-h))
        return pygame.Rect(upx, upy, w, h)
 
    def createRandomMap(self,n, points = []):
        self._obs=[]
        for i in range(0, n):
            self._obs.append(self.makeRandomRect())
        for p in points: self.removeObsPoint(p)

    def loadMap(self, rects, points = []):
        self._obs=[]
        for r in rects: self._obs.append(pygame.Rect(*r))
        for p in points: self.removeObsPoint(p)

    def removeObsPoint(self, point):
        '''removes all the obstacles  that enclose the point'''
        self._obs=[obs for obs in self._obs if not obs.collidepoint(point)]

    def draw(self):
        self.canvas.fill(white)
        for ob in self._obs:
            pygame.draw.rect(self.canvas,grey, ob)

    def draw_init_and_goal(self, init, goal):
        pygame.draw.circle(self.canvas, red, init, 3*node_rad, 3*node_rad)
        if goal: pygame.draw.circle(self.canvas, green, goal, 3*node_rad, 3*node_rad)

    def checkPoint(self, point):
        if point[0]<0 or point[0]>self._width or point[1]<0 or point[1]>self._heigh: return False
        for ob in self._obs:
            if ob.collidepoint(point):
                return False
        return True

    def checkSegment(self, p1, p2):
        for ob in self._obs:
            if ob.clipline(p1,p2):
                return False
        return True

    def stopping_configuration(self,p1, p2):
        n=int(1+p2distance(p1,p2)//local_planner_step)
        points = [ ( int((p2[0]*i + p1[0]*(n-i))/n) , int((p2[1]*i+p1[1]*(n-i))/n)) for i in range(0, n+1)]
        pp =  p1  
        for p in points: 
            if not self.checkPoint(p):
                return pp
        return p2

    def random_sample(self, theta = False):
        if theta: return int(random.uniform(0,self._width)), int(random.uniform(0,self._heigh)), int(random.uniform(0,360))
        return int(random.uniform(0,self._width)), int(random.uniform(0,self._heigh))