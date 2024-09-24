import pygame
from RRTbase import *
from Maps  import *



''' change the following values to see how the algorithm works'''
map_size = (1000, 600)
num_objects = 200
init = (50,100)
goal = (850,500)
dubbins_init = (*init, 0)
dubbins_goal = (*goal, 0)
Tree = TreeBase
def set_tree_type(ttype):
    global Tree
    Tree = ttype
class SQ_Planner:
    def __init__(self, map, init, goal):
        self.map=map
        self.init = init
        self.goal = goal
        self.iterations = 1
    def iterate(self, max_iter):
        pass
#####################RRT SIMPLE######################################    
class RRT(SQ_Planner):
    def __init__(self, map, init, goal):
        super().__init__(map, init, goal)
        self.tree = Tree(init, goal)
        self.tree.draw(map.canvas) 
    def iterate(self, max_iter):
        #alias
        tree=self.tree
        map= self.map
        goal=self.goal
        for i in range(max_iter):
            self.iterations+=1
            alpha = map.random_sample()
            if not self.iterations%100: alpha = goal 
            qn, edge = tree.nearest_to_swath(alpha)
            qs = map.stopping_configuration(qn, alpha)
            if qs != qn:
                tree.add_edge(qn, qs, edge, map.canvas)
            if qs == goal:
                tree.draw_path(map.canvas,goal)
                print("SUCCESS at iteration: ", self.iterations)
                return True
            #console iteration info
            print("Iteration: ", self.iterations)
        return False

def rrt_simple(map, init, goal):
    '''RRT SIMPLE WITH GOAL'''
    tree = Tree(init, goal)
    tree.draw(map.canvas)
    iterations = 1
    while(pygame_events()):
        alpha = map.random_sample()
        if not iterations%100: alpha = goal 
        qn, edge = tree.nearest_to_swath(alpha)
        qs = map.stopping_configuration(qn, alpha)
        if qs != qn:
            tree.add_edge(qn, qs, edge, map.canvas)
        if qs == goal:
            tree.draw_path(map.canvas,goal)
            print("SUCCESS at iteration: ", iterations)
            return True
        #console iteration info
        iterations +=1
        print("Iteration: ", iterations)
    return False
#####################RRT CONNECT######################################
class RRTconnect(SQ_Planner):
    def __init__(self, map, init, goal):
        super().__init__(map, init, goal)
        self.tree_a = Tree(init)
        self.tree_b = Tree(goal, color = (0,255,0))
        self.tree_a.draw(map.canvas)
        self.tree_b.draw(map.canvas)
    def iterate(self, max_iter):
        #alias
        tree_a, tree_b=self.tree_a, self.tree_b
        map= self.map
        for i in range(max_iter):
            self.iterations+=1
            alpha = map.random_sample()
            qn_a, edge_a = tree_a.nearest_to_swath(alpha)
            qs_a = map.stopping_configuration(qn_a, alpha)
            if qs_a != qn_a:
                tree_a.add_edge(qn_a, qs_a, edge_a, map.canvas)
                qn_b, edge_b = tree_b.nearest_to_swath(qs_a)
                qs_b = map.stopping_configuration(qn_b, qs_a)
                if qs_b != qn_b:
                    tree_b.add_edge(qn_b, qs_b, edge_b, map.canvas)
                if qs_b == qs_a:
                    tree_a.draw_path(map.canvas,qs_a)
                    tree_b.draw_path(map.canvas,qs_b)
                    print("SUCCESS at iteration: ", self.iterations)
                    return True
            #tree swapping. be careful with aliases
            if len(tree_a.tree) > len(tree_b.tree) : 
                self.tree_a, self.tree_b = self.tree_b, self.tree_a
                tree_a, tree_b = self.tree_a, self.tree_b
            print("Iteration: ", self.iterations)
        return False   
def rrt_connect(map, init, goal):
    '''RRT CONNECT: bidirectional search'''

    tree_a = Tree(init)
    tree_b = Tree(goal, color = (0,255,0))
    tree_a.draw(map.canvas)
    tree_b.draw(map.canvas)
    iterations = 1
    while(pygame_events()):
        alpha = map.random_sample()
        qn_a, edge_a = tree_a.nearest_to_swath(alpha)
        qs_a = map.stopping_configuration(qn_a, alpha)
        if qs_a != qn_a:
            tree_a.add_edge(qn_a, qs_a, edge_a, map.canvas)
            qn_b, edge_b = tree_b.nearest_to_swath(qs_a)
            qs_b = map.stopping_configuration(qn_b, qs_a)
            if qs_b != qn_b:
                tree_b.add_edge(qn_b, qs_b, edge_b, map.canvas)
            if qs_b == qs_a:
                tree_a.draw_path(map.canvas,qs_a)
                tree_b.draw_path(map.canvas,qs_b)
                print("SUCCESS at iteration: ", iterations)
                return True
        #console iteration info
        if len(tree_a.tree) > len(tree_b.tree) : 
            tree_a, tree_b = tree_b, tree_a
        iterations +=1
        print("Iteration: ", iterations)
        
#####################RRT dubbins car######################################       
class RRTdubbins(SQ_Planner):
    '''RRT Kino for dubbins SIMPLE WITH GOAL'''
    def __init__(self, map, init, goal):
        super().__init__(map, (*init,0), (*goal, 0))
        self.tree = TreeDubbins(self.init, self.goal)
        self.tree.draw(map.canvas)
    def iterate(self, max_iter):
        #alias
        tree=self.tree
        map= self.map
        goal=self.goal
        for i in range(max_iter):
            self.iterations+=1
            alpha = map.random_sample()
            if not self.iterations%100: alpha = goal 
            qn, edge = tree.nearest_to_swath(alpha)
            qs, tray = tree.steer(qn, alpha, map)
            if qs != qn:
                tree.add_edge(qn, qs, tray, map.canvas)
            if p2distance(qs, dubbins_goal)<dubbins_end_distance:
                tree.draw_path(map.canvas,qs)
                print("SUCCESS at iteration: ", self.iterations)
                return True
            print("Iteration: ", self.iterations)
        return False
    
def rrt_dubbins(map, init, goal):
    '''RRT Kino for dubbins SIMPLE WITH GOAL'''
    tree = TreeDubbins(dubbins_init, dubbins_goal)
    tree.draw(map.canvas)
    iterations = 1
    while(pygame_events()):
        alpha = map.random_sample()
        if not iterations%100: alpha = goal 
        qn, edge = tree.nearest_to_swath(alpha)
        qs, tray = tree.steer(qn, alpha, map)
        if qs != qn:
            tree.add_edge(qn, qs, tray, map.canvas)
        if p2distance(qs, dubbins_goal)<dubbins_end_distance:
            tree.draw_path(map.canvas,qs)
            print("SUCCESS at iteration: ", iterations)
            return True
        #console iteration info
        iterations +=1
        print("Iteration: ", iterations)
        
#####################RRT STAR######################################
  
class RRTstar(SQ_Planner):
    '''RRT STAR'''
    def __init__(self, map, init, goal):
        super().__init__(map, init, goal)
        self.tree = TreeStar(init, goal)
        self.tree.draw(map.canvas)

    def iterate(self, max_iter):
        #alias
        tree=self.tree
        map= self.map
        goal=self.goal
        repaint = False
        for i in range(max_iter):
            self.iterations+=1
            alpha = map.random_sample()
            if not self.iterations%100: alpha = goal 
            qn, edge = tree.nearest_to_swath(alpha)
            qs = map.stopping_configuration(qn, alpha)
            if qs != qn:
                Q_near = tree.get_closests_nodes(qs,optimal_radius(len(tree.tree))) #<- (node, distance, cost, dist+cost)
                dmin= p2distance(qs,qn)
                if edge: cmin = dmin+tree.node_cost[tree.tree[edge]]+p2distance(tree.tree[edge],qn)
                else: cmin= dmin+tree.node_cost[qn]
                #first strategy
                for qi in Q_near:
                    if qi[3]<cmin and map.checkSegment(qi[0],qs):
                        tree.add_edge(qi[0],qs,None,map.canvas)
                        Q_near.remove(qi)
                        break
                else: tree.add_edge(qn, qs, edge, map.canvas)
                #second strategy : rewiring
                cmin = tree.node_cost[qs]
                for qi in Q_near:
                    if cmin+qi[1]<qi[2] and map.checkSegment(qi[0],qs):
                        tree.change_parent(qi[0],qs)
                        repaint = True
            if qs == goal:
                tree.draw_path(map.canvas,goal)
                print("SUCCESS at iteration: ", self.iterations)
                #return True
            #repainting managing
            if(repaint):
                map.draw()
                tree.draw(map.canvas)
                repaint = False
            #console iteration info
            #print("Iteration: ", self.iterations)
            if goal in tree.tree:
                print("lenght {0} at iter{1}: ".format(tree.node_cost[goal], self.iterations))
            else:
                print("Iteration: ", self.iterations)
        return False
    
def rrt_star(map, init, goal):
    '''RRT STAR WITH GOAL'''
    tree = TreeStar(init, goal)
    tree.draw(map.canvas)
    iterations = 1
    repaint = False
    while(True):
        event = pygame_events([pygame.KEYDOWN])
        if not event: break
        if event == pygame.KEYDOWN: return True
        alpha = map.random_sample()
        if not iterations%100: alpha = goal 
        qn, edge = tree.nearest_to_swath(alpha)
        qs = map.stopping_configuration(qn, alpha)
        if qs != qn:
            Q_near = tree.get_closests_nodes(qs,optimal_radius(len(tree.tree))) #<- (node, distance, cost, dist+cost)
            dmin= p2distance(qs,qn)
            if edge: cmin = dmin+tree.node_cost[tree.tree[edge]]+p2distance(tree.tree[edge],qn)
            else: cmin= dmin+tree.node_cost[qn]
            #first strategy
            for qi in Q_near:
                if qi[3]<cmin and map.checkSegment(qi[0],qs):
                    tree.add_edge(qi[0],qs,None,map.canvas)
                    Q_near.remove(qi)
                    break
            else: tree.add_edge(qn, qs, edge, map.canvas)
            #second strategy : rewiring
            cmin = tree.node_cost[qs]
            for qi in Q_near:
                if cmin+qi[1]<qi[2] and map.checkSegment(qi[0],qs):
                    tree.change_parent(qi[0],qs)
                    repaint = True
        if qs == goal:
            tree.draw_path(map.canvas,goal)
            print("SUCCESS with lenght {0} at iter{1}: ".format(
                tree.node_cost[goal], self.iterations))
            #return True
        #repainting managing
        if(repaint):
            map.draw()
            tree.draw(map.canvas)
            repaint = False
        #console iteration info
        iterations +=1
        
        if goal in tree.tree:
            print("lenght {0} at iter{1}: ".format(tree.node_cost[goal], iterations))
        else:
            print("Iteration: ", iterations) 
            

###############Informed RRT STAR######################################
from math import cos, sin , atan2, degrees
import random
def draw_rotated_ellipse(surface, color, center, a, b, angle):
    target_rect = pygame.Rect(center[0]-a, center[1]-b,2*a,2*b)
    ellipse_surface = pygame.Surface(target_rect.size, pygame.SRCALPHA)
    pygame.draw.ellipse(ellipse_surface, color, (0, 0, 2*a, 2*b),4)
    rotated_surface = pygame.transform.rotate(ellipse_surface, -degrees(angle))
    rect = rotated_surface.get_rect(center = target_rect.center)
    surface.blit(rotated_surface, rect)
def uniform_random_circle_point():
    r=random.uniform(0,1)**0.5
    th = random.uniform(0,2*math.pi)
    return r*cos(th), r*sin(th)
class informedRRTstar(SQ_Planner):
    '''informed RRT STAR'''
    def __init__(self, map, init, goal):
        super().__init__(map, init, goal)
        self.tree = TreeStar(init, goal)
        self.tree.draw(map.canvas)
        #elipsoid invariants computation
        self.c_min = p2distance(goal,init)
        self.q_center=(0.5*(goal[0]+init[0]),0.5*(goal[1]+init[1]))
        #best cost
        self.ang = ang = atan2(goal[1]- init[1],goal[0]-init[0])
        self.Rot = ((cos(ang), -sin(ang)),(sin(ang), cos(ang)))
        self.c_best=None
        
    def sample(self, c_max):
        map= self.map
        R = self.Rot
        if c_max:
            while True:
                r1=0.5*c_max
                r2=0.5*(c_max**2-self.c_min**2)**0.5
                q_ball = uniform_random_circle_point()
                q_rand = (self.q_center[0]+r1*q_ball[0]*R[0][0]+r2*q_ball[1]*R[0][1],
                         self.q_center[1]+r1*q_ball[0]*R[1][0]+r2*q_ball[1]*R[1][1])
                if (0 < q_rand[0] < map._width) and (0 < q_rand[1] < map._heigh):
                    return q_rand
        return map.random_sample()
        
    def draw_ellipsoid(self, canvas):
        if not self.c_best: return
        r1=0.5*self.c_best
        r2=0.5*(self.c_best**2-self.c_min**2)**0.5
        draw_rotated_ellipse(canvas, (100, 255, 100), self.q_center,
                             r1, r2, self.ang)
        pygame.draw.circle(canvas, (100, 255, 100), self.q_center, 3*node_rad, 3*node_rad)

       

    def iterate(self, max_iter):
        #alias
        tree=self.tree
        map= self.map
        goal=self.goal
        repaint = False
        for i in range(max_iter):
            self.iterations+=1
            alpha = self.sample(self.c_best)
            if not self.iterations%100: alpha = goal 
            qn, edge = tree.nearest_to_swath(alpha)
            qs = map.stopping_configuration(qn, alpha)
            if qs != qn:
                Q_near = tree.get_closests_nodes(qs,optimal_radius(len(tree.tree))) #<- (node, distance, cost, dist+cost)
                dmin= p2distance(qs,qn)
                if edge: cmin = dmin+tree.node_cost[tree.tree[edge]]+p2distance(tree.tree[edge],qn)
                else: cmin= dmin+tree.node_cost[qn]
                #first strategy
                for qi in Q_near:
                    if qi[3]<cmin and map.checkSegment(qi[0],qs):
                        self.draw_ellipsoid(map.canvas)
                        tree.add_edge(qi[0],qs,None,map.canvas)
                        Q_near.remove(qi)
                        break
                else:
                    self.draw_ellipsoid(map.canvas)
                    tree.add_edge(qn, qs, edge, map.canvas)
                    
                
                #second strategy : rewiring
                cmin = tree.node_cost[qs]
                for qi in Q_near:
                    if cmin+qi[1]<qi[2] and map.checkSegment(qi[0],qs):
                        tree.change_parent(qi[0],qs)
                        repaint = True
            if qs == goal:
                self.draw_ellipsoid(map.canvas)
                tree.draw_path(map.canvas,goal)
                self.c_best = tree.node_cost[goal]
                print("SUCCESS with lenght {0} at iter{1}: ".format(self.c_best, self.iterations))
                #return True
            #repainting managing
            if(repaint):
                map.draw()
                self.draw_ellipsoid(map.canvas) 
                tree.draw(map.canvas)
                repaint = False
              
            #console iteration info
            if self.c_best:
                print("Iteration: {0} Length:{1:.9}/{2:.9}".format(self.iterations,
                                    self.c_best, self.c_min))
            else:
                print("Iteration: ", self.iterations)
        
        return False
    
menu = '''
PLANNER DEMOS - Miguel Hernando
Press any key to start:
0 - Display Menu
1 - RRT simple [default]
2 - RRT-Connect 
3 - RRT*
4 - Dubbins RRT
5 - Use Continuous Trees
6 - Use Discretized Trees (1,2)
7 - Load a Random map
8 - Load Map 1
9 - Load Map 2
A - Load Map without solution
'''


import os
w, h = map_size

if __name__ == '__main__':
    
 


    pygame.init()
    screen = pygame.display.set_mode((w,h))
    
   
    map = BaseMap(*map_size) 
    map.loadMap(map1, [init,goal])
    planner = rrt_simple
    map.draw()
    map.draw_init_and_goal(init,goal)
    pygame_print_text(map.canvas, (100,80), menu, 20)
    pygame.display.update()

    
    while(True):
        key = pygame_wait_for_key()
        if key != pygame.K_0: map.draw()
        if key == pygame.QUIT: break
        if key == pygame.K_1: planner = rrt_simple
        if key == pygame.K_2: planner = rrt_connect
        if key == pygame.K_3: planner = rrt_star
        if key == pygame.K_4: planner = rrt_dubbins
        if key == pygame.K_5: Tree = TreeBase
        if key == pygame.K_6: Tree = TreeDiscretized
        if key == pygame.K_7: 
            map.createRandomMap(num_objects, [init,goal])
            map.draw()
            map.draw_init_and_goal(init,goal)
            pygame.display.update()

            continue
        if key == pygame.K_8: 
            map.loadMap(map1, [init,goal])
            map.draw()
            map.draw_init_and_goal(init,goal)
            pygame.display.update()
            continue
        if key == pygame.K_9: 
            map.loadMap(map2, [init,goal])
            map.draw()
            map.draw_init_and_goal(init,goal)
            pygame.display.update()
            continue
        if key == pygame.K_a: 
            map.loadMap(map3, [init,goal])
            map.draw()
            map.draw_init_and_goal(init,goal)
            pygame.display.update()
            continue
        if key == pygame.K_0: 
            pygame_print_text(map.canvas, (100,80), menu, 20)
            continue
        if not planner(map, init, goal): break
    pygame.quit()
