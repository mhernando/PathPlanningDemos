import pygame
from RRTbase import *
from maps  import *

''' change the following values to see how the algorithm works'''
map_size = (1000, 600)
num_objects = 200
init = (50,100)
goal = (850,500)
dubbins_init = (*init, 0)
dubbins_goal = (*goal, 0)
Tree = TreeBase

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
            print("SUCCESS at iteration: ", iterations)
            #return True
        #repainting managing
        if(repaint):
            map.draw()
            tree.draw(map.canvas)
            repaint = False
        #console iteration info
        iterations +=1
        print("Iteration: ", iterations)


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
'''        
if __name__ == '__main__':
    pygame.init()
    map = BaseMap(*map_size) 
    map.loadMap(map1, [init,goal])
    planner = rrt_simple
    map.draw()
    map.draw_init_and_goal(init,goal)
    pygame_print_text(map.canvas, (100,100), menu, 20)
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
        if key == pygame.K_0: 
            pygame_print_text(map.canvas, (100,100), menu, 20)
            continue
        if not planner(map, init, goal): break
    pygame.quit()
