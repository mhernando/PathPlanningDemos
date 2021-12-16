'''file that includes the basic components for RRT algorithms: 
drawables trees'''

import math
import pygame
from Definitions import *
from Maps import *




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
    def dubbins_distance(self, p1, p2):
        v=[p2[0]-p1[0],p2[1]-p1[1]]
        if v[0]**2+v[1]**2 < 0.01: return 0
        m= module(v)
        v[0],v[1] = v[0]/m, v[1]/m
        angle = math.radians(p1[2]) - math.atan2(v[1],v[0])
        return m*(2-math.cos(angle))
    def nearest_to_swath(self, p):
        qn =  self.root
        min_dist = self.dubbins_distance(qn,p)
        for p2 in self.tree:
            d = self.dubbins_distance(p2,p)
            if d < min_dist:
                min_dist, qn = d, p2
        return qn, None
        

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
        
