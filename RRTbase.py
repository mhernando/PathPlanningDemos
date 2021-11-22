'''file that includes the basic components for RRT algorithms: 
drawables maps and drawables trees'''

import random
import math
import pygame
from pygame import draw

#constants
#Colors
grey = (70, 70,70)
blue = (0, 0, 255)
green = (0, 255,0)
red = (255,0,0)
white = (255, 255,255)

#environment
min_obs_size = 10
obs_variance = 50
local_planner_step = 3
edge_discretization_step = 10

#graphics
node_rad = 2
edge_thickness = 2
edge_color = blue
node_color = blue

#dubbins
dubbins_advance_step = 3 #pixels
dubbins_rotation_step = 8 #degs
dubbins_end_distance  = (360/dubbins_rotation_step)*dubbins_advance_step/7


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

class TreeBase:
    def __init__(self, root, goal = None , color = edge_color):
        self.root = root 
        self.goal = goal
        self.color = color
        self.tree = {} #dict where each node stores his parent

    def draw(self, canvas):
        pygame.draw.circle(canvas, red, self.root[:2], 3*node_rad, 3*node_rad)
        if self.goal: pygame.draw.circle(canvas, green, self.goal[:2], 3*node_rad, 3*node_rad)
        for p2 in self.tree:
            self.draw_edge(canvas,p2)
        if self.goal in self.tree:
            self.draw_path(canvas, self.goal)

    def draw_edge(self, canvas, p2):
        if not p2 in self.tree: return
        pygame.draw.line(canvas, self.color, self.tree[p2][:2] ,p2[:2], edge_thickness)
        pygame.draw.circle(canvas, node_color , p2[:2], node_rad, node_rad)
        pygame.draw.circle(canvas, node_color, self.tree[p2][:2], node_rad, node_rad)

    def draw_path(self, canvas, p2):
        while(p2 in self.tree):
            pygame.draw.line(canvas, red, self.tree[p2][:2] ,p2[:2], 2*edge_thickness)
            p2 = self.tree[p2]
        pygame.display.update()

    def add_edge(self, p1,p2,p3=None, canvas = None): 
        '''p2 has the parent p1. if p2 exists, nothing is done. if edge, 
        then p1 is considered the intermediate point of the edge p3 and its parent'''
        tree = self.tree
        if p2 in tree: return False
        if p3 and (p3 in self.tree) :
            if p1 in tree:
                tree[p2] = p1
            else:
                tree[p1], tree[p3] ,  tree[p2] , = tree[p3], p1 , p1     
        else:
            tree[p2] = p1
        if canvas:
            self.draw_edge(canvas, p2)
        return True

    def nearest_to_swath(self, p):
        qn =  self.root
        min_dist = p2distance(qn,p)
        edge = None
        for p2, p1 in self.tree.items():
            d,q= distance_point_to_segment(p,p1,p2)
            if d < min_dist:
                min_dist, qn = d, q
                if (qn == p1) or (qn == p2): edge = None
                else: edge = p2
        return qn, edge

class TreeDiscretized(TreeBase):
    def add_edge(self, p1,p2,p3=None, canvas = None): 
        '''p2 has p1 as parent. if p2 exists, nothing is done. 
        p3 is included for compatibility but not used in the discretized version
        a set of intermediate edges is included between p2 and p1'''
        tree = self.tree
        if p2 in tree: return False
        
        n=int(p2distance(p1,p2)//edge_discretization_step)
        pp = p2
        if n:
            nodes = [ ( int((p1[0]*i + p2[0]*(n-i))/n) , int((p1[1]*i+p2[1]*(n-i))/n)) for i in range(1, n)]
            for p in nodes: tree[pp], pp = p,p
        tree[pp]=p1 

        if canvas:
            pygame.draw.line(canvas, self.color, p1 ,p2, edge_thickness)
            pygame.draw.circle(canvas, node_color , p2, node_rad, node_rad)
            pygame.draw.circle(canvas, node_color, p1, node_rad, node_rad)
            if n>1: 
                for p in nodes: pygame.draw.circle(canvas, grey, p, node_rad, node_rad)
        return True

    def nearest_to_swath(self, p):
        qn =  self.root
        min_dist = p2distance(qn,p)
        for p2 in self.tree:
            d = p2distance(p2,p)
            if d < min_dist:
                min_dist, qn = d, p2
        return qn, None
def dubbins_action(p,action):
    angle = math.radians(p[2]+action[1]*dubbins_rotation_step)
    return p[0]+dubbins_advance_step*math.cos(angle), p[1]+dubbins_advance_step*math.sin(angle), (360+math.degrees(angle))%360

class TreeDubbins(TreeDiscretized):
    def __init__(self, root, goal = None , color = edge_color):
        super().__init__(root,goal, color)
        self.node_theta = {root:0} #dict where each node stores its orientation
        self.node_tray = {} #dict that stores the edge trayectory
    def steer(self,q1,q2,map): 
        '''advances with one action until the distance increases or collides
        returns : 1) the last valid q if collision, or the last point,
                  2) a secquence of points that describe the tray'''
        actions=((1,-1),(1,0),(1,1)) #advance and rotation
        dist = [p2distance(dubbins_action(q1,action),q2) for action in actions]
        action = actions[dist.index(min(dist))] #select the actions whose first step minimizes the distance
        qs = q1; tray = []; q = dubbins_action(q1,action); min_dist = p2distance(q1,q2)
        dist = p2distance(q,q2)
        while dist<min_dist:       
            if not map.checkPoint(q[:2]): break 
            qs, min_dist = q, dist
            tray.append(qs)
            q = dubbins_action(qs,action)
            dist = p2distance(q,q2)
        return qs, tray

        

    def add_edge(self, q1, q2, tray, canvas = None):
        pp=q1
        for p in tray: self.tree[p], pp = pp, p
        if canvas:
            pp=q1
            for p in tray:
                pygame.draw.line(canvas, self.color, pp[:2] ,p[:2], edge_thickness)
                pp=p
            pygame.draw.circle(canvas, node_color , q1[:2], node_rad, node_rad)
            pygame.draw.circle(canvas, node_color, q2[:2], node_rad, node_rad)


class TreeStar(TreeBase):
    def __init__(self, root, goal = None , color = edge_color):
        super().__init__(root,goal, color)
        self.node_cost = {root:0} #dict where each node stores its acumulated cost
        self.edge_cost = {} #dict where each node stores the cost from parent (distance)

    def compute_cost(self, p):
        if not p in self.tree: return
        self.edge_cost[p] = p2distance(self.tree[p],p)
        self.node_cost[p] = self.edge_cost[p]+self.node_cost[self.tree[p]]

    def add_edge(self, p1,p2,p3=None, canvas = None): 
        ret=super().add_edge(p1,p2,p3,canvas)
        if p3: self.compute_cost(p1)
        self.compute_cost(p2)
        if p3: self.compute_cost(p3)
        return ret

    def update_cost(self, p, parent_changed= False):
        '''the parent cost of p has changed, so its costs should be updated
        if parent changed is true, also de edge to parent was modified'''
        if not p in self.tree: return
        if parent_changed: self.edge_cost[p] = p2distance(self.tree[p],p)
        self.node_cost[p] = self.edge_cost[p]+self.node_cost[self.tree[p]]
        #get those nodes that has p has parent
        children = [node for node, parent in self.tree.items() if parent == p]
        for child in children: self.update_cost(child)

    def get_closests_nodes(self,p,r):
        '''retrieves a list of tuples with the (node, distance to p, cost, cost + distance) at a distance to p less than r
        to speed up the algorithm the list is sorted by the minimun cost+distance'''
        nodes = []
        for n in self.tree:
            d = p2distance(p,n)
            if d<r: nodes.append((n,d,self.node_cost[n], d+self.node_cost[n]))
        nodes.sort(key=lambda x:x[3])
        return nodes

    def change_parent(self, p, new_parent):
        if not p in self.tree: return
        self.tree[p]=new_parent
        self.update_cost(p, True)

'''utility pygame functions'''
def pygame_events(event_list = []):
    pygame.display.update()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
        for e in event_list:
            if event.type == e: return  e
    return True
def pygame_wait_for_key():
    while(True):
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                return event.key
            if event.type == pygame.QUIT:
                return pygame.QUIT
def pygame_print_text(canvas, pos, text, size):
    font = pygame.font.Font(pygame.font.get_default_font(), size)
    h=0
    for line in text.splitlines():
        text_surface = font.render(line, True, (0, 0, 0))
        canvas.blit(text_surface,dest=(pos[0],pos[1]+h))
        h+=size*3//2
    pygame.display.update()

#main for testing
if __name__ == '__main__':
    from time import sleep
    pygame.init()
    tree = TreeBase((500,300))
    map = BaseMap(1000, 600) 
    map.createRandomMap(50)
    map.removeObsPoint(tree.root)

    map.draw()
    tree.draw(map.canvas)
    
    while(pygame_events()):
        alpha = map.random_sample()
        qn, edge = tree.nearest_to_swath(alpha)
        qs = map.stopping_configuration(qn, alpha)
        if qs != qn:
            tree.add_edge(qn, qs, edge, canvas = map.canvas)
        
