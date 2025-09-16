'''this file implements the common map operations'''

import random
import math
import pygame
import json
from Definitions import *

def_init = (50,100)
def_goal = (850,500)
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
map4=((250,200,400,20),
    (300,200,20,300),
    (250,500,300,20))
pared = 10
map5=((500,0, pared, 60),
        (200,100,600,pared),
        (500,100,pared,300),
        (200,400,600,pared))
map6 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (0,95,75,10),     # Habitacion 1
         (135,95,65,10),  # Habitacion 1
         (200,95,10,155),  # Habitacion 1
         (0,245,200,10),   # Habitacion 1/2
         (200,250,10,155), # Habitacion 2
         (0,395,75,10),   # Habitacion 2
         (135,395,65,10),  # Habitacion 2
         (0,495,185,10),   # Habitacion 3
         (245,495,10,105), # Habitacion 3/4
         (255,495,180,10), # Habitacion 4
         (495,495,10,105), # Habitacion 4/5
         (565,495,180,10), # Habitacion 5
         (745,495,10,105), # Habitacion 5/6
         (815,495,185,10), # Habitacion 6
         (800,395,200,10), # Habitacion 7
         (800,245,10,90), # Habitacion 7
         (800,245,200,10), # Habitacion 7/8
         (800,165,10,90), # Habitacion 8
         (800,95,200,10),  # Habitacion 8
         (300,125,170,30), # Centro arriba izquierda
         (300,125,30,100), # Centro arriba izquierda
         (530,125,170,30), # Centro arriba derecha
         (670,125,30,100), # Centro arriba derecha
         (300,354,170,30), # Centro abajo izquierda
         (300,284,30,100), # Centro abajo izquierda
         (530,354,170,30), # Centro abajo derecha
         (670,284,30,100), # Centro abajo derecha
)
# MAPA PLANTA 2
map7 = (
         (0,0,1000,10),    # Borde superior
         (0,0,10,600),     # Borde izquierda
         (990,0,10,600),   # Borde derecha
         (0,590,1000,10),  # Borde inferior
         (0,240,150,10),
         (0,350,45,10),
         (105,350,45,10),
         (140,240,10,120),
         (245,355,10,245),
         (245,0,10,160),
         (245,220,10,80),
         (245,150,100,10),
         (245,290,100,10),
         (340,90,10,210),
         (340,90,60,10),
         (390,0,10,40),
         (390,90,10,200),
         (460,280,10,100),
         (460,280,270,10),
         (720,0,10,30),
         (720,90,10,290),
         (460,370,55,10),
         (565,370,425,10),
         (720,180,155,10),
         (935,180,65,10),

         (545,430,120,10),
         (545,430,10,170),
         (655,490,10,110),
)
class MapData:
    def __init__(self, map, description, init = def_init, goal = def_goal):
        self.map = map
        self.init = init
        self.goal = goal
        self.description = description
        
maps=(MapData(map1,"Mapa 'J' Map"),
      MapData(map2,"Easy Map"),
      MapData(map3,"Imposible Map"),
      MapData(map4,"T Map"),
      MapData(map5,"Mapa NIRT",(200,200),(800,200)),
      MapData(map6, "Plano 1", (106, 170), (868, 548)),
      MapData(map7, "Plano 2", (30, 570), (790, 340))
      )

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
        self._font=pygame.font.SysFont("Arial", 24)
  
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

    def save_map_to_file(self, filename, init, goal):
        """Guarda el mapa en un archivo JSON incluyendo obstáculos, punto inicial y final."""
        data = {
            "obstacles": [list(rect) for rect in self._obs],  # convertir pygame.Rect a lista
            "init": list(init),
            "goal": list(goal)
        }
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

    def load_map_from_file(self, filename):
        """Carga el mapa desde un archivo JSON, incluyendo obstáculos, punto inicial y final."""
        with open(filename, 'r') as f:
            data = json.load(f)
        rects = [tuple(r) for r in data.get("obstacles", [])]
        self.loadMap(rects)
        init = tuple(data.get("init", ()))
        goal = tuple(data.get("goal", ()))
        return init, goal

    def removeObsPoint(self, point):
        '''removes all the obstacles  that enclose the point'''
        self._obs=[obs for obs in self._obs if not obs.collidepoint(point)]

    def show_text(self, text):       
        text_surface = self._font.render(text, True, (255, 255, 255))
        text_rect = text_surface.get_rect()
        text_bkg = pygame.Rect(self._width-300,10,290,30)
        
        text_rect.right = text_bkg.right - 10
        text_rect.top = text_bkg.top + (text_bkg.height - text_rect.height) // 2
        self.canvas.fill((30, 30, 80), text_bkg)
        self.canvas.blit(text_surface, text_rect)
        
    def draw(self):
        self.canvas.fill(white)
        for ob in self._obs:
            pygame.draw.rect(self.canvas,grey, ob)

    def draw_init_and_goal(self, init, goal):
        pygame.draw.circle(self.canvas, red, init, end_node_rad, end_node_rad)
        if goal: pygame.draw.circle(self.canvas, green, goal, end_node_rad, end_node_rad)

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
            pp=p
        return p2

    def random_sample(self, theta = False):
        if theta: return int(random.uniform(0,self._width)), int(random.uniform(0,self._heigh)), int(random.uniform(0,360))
        return int(random.uniform(0,self._width)), int(random.uniform(0,self._heigh))