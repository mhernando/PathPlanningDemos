import pygame
from RRTbase import *
from Maps  import *
from RRTdemos import *
class ellip():
    def __init__(self, q1, q2):
        self.q1=q1
        self.q2=q2
        self.c_min = p2distance(q2,q1)
        self.q_center = (0.5*(q2[0]+q1[0]),0.5*(q2[1]+q1[1]))
        self.ang = ang = atan2(q2[1]- q1[1],q2[0]-q1[0])
        self.Rot = ((cos(ang), -sin(ang)),(sin(ang), cos(ang)))
class n_informedRRTstar(SQ_Planner):
    '''informed RRT STAR'''
    def __init__(self, map, init, goal):
        super().__init__(map, init, goal)
        self.tree = TreeStar(init, goal)
        self.tree.draw(map.canvas)
        self.c_best=None
        #include the info for the init-goal ellipsoid
        el = ellip(init,goal)
        self.ellipses={init:{goal:el}}
        self.c_min=el.c_min
        self.vol = map._width*map._heigh
        self.n_nodes=1 #number of normalized tree nodes
    def get_ellip(self, q1, q2):
        if q1 in self.ellipses :
            if q2 in self.ellipses[q1]:
                return self.ellipses[q1][q2]
        el = ellip(q1,q2)
        self.ellipses[q1] = {q2:el}
        return el
    #it will work only if there is a path to goal
    def pick_subpath(self):
        #de momento la mitad de las veces toma el total
        if random.choice([True, False]):return self.init, self.goal
        path =  self.tree.get_path(self.goal)
        while(True):
            i1, i2 = sorted(random.sample(range(len(path)), 2))
            if i1 < i2 :return path[i1] , path[i2]
        
    def sample(self):
        #alias
        map= self.map
        costs =  self.tree.node_cost
        if not self.c_best: return map.random_sample(),1
        #there is a solution so pick a sub_path 
        q1 , q2 = self.pick_subpath()
        el = self.get_ellip(q1 , q2)
        c_max = costs[q2] - costs[q1]
        R = el.Rot
        r1=0.5*c_max
        if c_max < el.c_min : return map.random_sample(), 1
        r2=0.5*(c_max**2-el.c_min**2)**0.5
        while True:
            q_ball = uniform_random_circle_point()
            q_rand = (el.q_center[0]+r1*q_ball[0]*R[0][0]+r2*q_ball[1]*R[0][1],
                     el.q_center[1]+r1*q_ball[0]*R[1][0]+r2*q_ball[1]*R[1][1])
            if (0 < q_rand[0] < map._width) and (0 < q_rand[1] < map._heigh):
                    return q_rand, r1*r2/self.vol
        
        
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
            alpha, vol = self.sample()
            if not self.iterations%100: alpha = goal 
            qn, edge = tree.nearest_to_swath(alpha)
            qs = map.stopping_configuration(qn, alpha)
            if qs != qn:
                #number of nodes have to be normalized with the generation area
                Q_near = tree.get_closests_nodes(qs,optimal_radius(self.n_nodes)) #<- (node, distance, cost, dist+cost)
                dmin= p2distance(qs,qn)
                if edge: cmin = dmin+tree.node_cost[tree.tree[edge]]+p2distance(tree.tree[edge],qn)
                else: cmin= dmin+tree.node_cost[qn]
                #first strategy
                for qi in Q_near:
                    if qi[3]<cmin and map.checkSegment(qi[0],qs):
                        #self.draw_ellipsoid(map.canvas)
                        tree.add_edge(qi[0],qs,None,map.canvas)
                        self.n_nodes+=vol
                        Q_near.remove(qi)
                        break
                else:
                    #self.draw_ellipsoid(map.canvas)
                    tree.add_edge(qn, qs, edge, map.canvas)
                    self.n_nodes+=vol #normalized number of nodes
                    
                
                #second strategy : rewiring
                cmin = tree.node_cost[qs]
                for qi in Q_near:
                    if cmin+qi[1]<qi[2] and map.checkSegment(qi[0],qs):
                        tree.change_parent(qi[0],qs)
                        repaint = True
            if qs == goal:
                #self.draw_ellipsoid(map.canvas)
                tree.draw_path(map.canvas,goal)
                self.c_best = tree.node_cost[goal]
                print("SUCCESS with lenght {0} at iter{1}: ".format(self.c_best, self.iterations))
                #return True
            #repainting managing
            if(repaint):
                map.draw()
                #self.draw_ellipsoid(map.canvas) 
                tree.draw(map.canvas)
                repaint = False
              
            #console iteration info
            if self.c_best:
                print("Iteration2: {0} Length:{1:.9}/{2:.9}".format(self.iterations,self.c_best, self.c_min))
            else:
                print("Iteration2: ", self.iterations)
                pass
        
        return False

import os

def log_info(planner, log):
    if planner.c_best:
        log.append((planner.iterations, planner.c_best, planner.c_best/planner.c_min))
if __name__ == '__main__':
    
    w, h = map_size
    map_size = (1000, 600)
    init = (50,100)
    goal = (850,500)
    



    pygame.init()
    screen = pygame.display.set_mode((w,h))
    
   
    mapa = BaseMap(*map_size)
    mapa.createRandomMap(num_objects, [init,goal])
    mapa.loadMap(map1, [init,goal]) #map4,  map1
    
    
    planner = informedRRTstar(mapa, init, goal)
    planner2 = n_informedRRTstar(mapa, init, goal)
    mapa.draw()
    mapa.draw_init_and_goal(init,goal)
    pygame.display.update()
    run = True
    log1=[]
    log2=[]
    while(run):
        planner.iterate(20)
        planner2.iterate(20)
        log_info(planner, log1)
        log_info(planner2, log2)
        pygame.display.update()
        
        ev = pygame.event.get()
        for event in ev:
            if event.type == pygame.KEYDOWN or event.type == pygame.QUIT:
                run = False
        

    #print(planner.tree.get_path(goal)) 
    pygame.quit()
    import matplotlib.pyplot as plt
    i1, c1, cr1 = zip(*log1)
    i2, c2, cr2 = zip(*log2)
    plt.plot(i1, cr1, label='informed RRT*', color='blue')
    plt.plot(i2, cr2, label='N-informed RRT*', color='green')
    plt.show()