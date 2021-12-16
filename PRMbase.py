'''file that includes the basic components for RRT algorithms: 
drawables trees'''

import math
import pygame
from Definitions import *
from Maps import *

class Node:
    def __init__(self, state, g, h, parent):
        self.state=state
        self.g=g
        self.h=h
        self.f=g+h
        self.parent=parent
    def __eq__(self, other):
        return self.state == other.state
    def __ne__(self, other):
        return self.state != other.state
    def __repr__(self):
        return "Node value f: %d  "%self.f

class Astar:
    def __init__(self,heuristics = None):
        self.closed=[]
        self.open=[]
        self.init=None
        self.goal=None
        if heuristics: self.f_heur=heuristics
        else:  self.f_heur=self.default_heur

    def default_heur(self,state, goal):
        return p2distance(state,goal)

    def heur_dijkstra(self, state, goal):
        return 0
    
    def f_edge_cost(self, state1, state2):
        if not state1: return 0   
        return p2distance(state1,state2)

    def get_childs(self,n_state, graph):
        v=graph.get_childs(n_state.state)
        if n_state.parent and n_state.parent.state in v: v.remove(n_state.parent.state)
        childs = [Node(n,n_state.g+self.f_edge_cost(n_state.state,n),self.f_heur(n,self.goal),n_state) for n in v ]
        return childs

    @staticmethod
    def obtainPathTo(n_state):
        secuence=[]
        while n_state:
            secuence.append(n_state.state)
            n_state=n_state.parent
        return secuence[::-1]

    def solve(self, init, goal, graph):
        self.init=init
        self.goal=goal
        self.closed=[]
        f_h=self.f_heur
        self.open=[Node(init,0,f_h(init,goal),None)]

        while self.open:
            print("explored: "+str(len(self.closed)) + " Alive: "+ str(len(self.open))+
                  "best f: "+str(self.open[0].f))
            #ordeno abiertos
            self.open.sort(key=lambda o: o.f)
            # extraigo el mejor y lo paso a cerrados
            current=self.open.pop(0)
            self.closed.append(current)
            if current.state==goal:
                return self.obtainPathTo(current)
            for n in self.get_childs(current, graph):
                #si ya explorado paso de el
                if n in self.closed: continue
                #calculo su g como +1 al actual
                if n in self.open: #el nodo existe, actualizo la info si este es mejor
                    ind=self.open.index(n)
                    if self.open[ind].f > n.f: self.open[ind]=n
                else: #es un nuevo nodo, lo agrego a los vivos
                    self.open.append(n)
        return None


class PRMBase:
    def __init__(self):
        self.vertices = {} #dict with vertices. Each vertex has a list of conected nodes. A node is a tuple of a pair of integers 
        
    def draw(self, canvas):
        for v, edges in self.vertices.items():
            for p in edges:
                pygame.draw.line(canvas, yellow, v ,p, edge_thickness)
        for v in self.vertices: pygame.draw.circle(canvas, node_color , v, node_rad, node_rad)   
    def draw_path(self,path, canvas):
        pi = path[0]
        for pf in path[1:]:
            pygame.draw.line(canvas, red, pi ,pf, 2*edge_thickness)
            pi=pf
        
    def addVertex(self, qn, map):
        if qn in self.vertices: return False      
        if not map.checkPoint(qn): return False
        self.vertices[qn]=[]
        l=self.get_closests_nodes(qn,prm_proximity_radius)[:prm_num_connections]
        for qi in l:
            if map.checkSegment(qn, qi[0]): self.addEdge(qn,qi[0])
        return True

    def addEdge(self, v1,v2):
        if v1 not in self.vertices: self.vertices[v1]=[]
        if v2 not in self.vertices: self.vertices[v2]=[]
        self.vertices[v1].append(v2)
        self.vertices[v2].append(v1)

    def get_closests_nodes(self,p,r):
        '''retrieves a list of tuples with vertexes and the distance to p less than r
        sorted by the minimun distance'''
        nodes = []
        for n in self.vertices:
            d = p2distance(p,n)
            if d<r: nodes.append((n,d))
        nodes.sort(key=lambda x:x[1])
        return nodes
    
    def populate(self, map, N=100):
        i=0
        while i<N:
            qn = map.random_sample()
            if not self.addVertex(qn,map): continue
            i+=1
    def get_childs(self, v):
        return self.vertices[v]
    def clear(self):
        self.vertices={}

#main for testing
if __name__ == '__main__':
    
    init = (50,100)
    goal = (850,500)
    pygame.init()
    prm = PRMBase()
    map = BaseMap(1000, 600) 
    map.createRandomMap(150, [init,goal])
    prm.draw(map.canvas)
    map.draw()
    
    while(pygame_events() and len(prm.vertices)<2000):
        prm.populate(map,100)
        prm.draw(map.canvas)
    
    if not prm.addVertex(init,map): print("Inicio no conectable: SIN SOLUCION")
    if not prm.addVertex(goal, map): print("Fin no conectable: SIN SOLUCION")
    prm.draw(map.canvas)
    map.draw_init_and_goal(init,goal)
    solver = Astar()
    sol=solver.solve(init,goal,prm)
    if sol: prm.draw_path(sol,map.canvas)
    else: print("NO SE ENCONTRO")
    while(pygame_events()):
        pass 

        