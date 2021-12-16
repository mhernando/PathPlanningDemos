import pygame
from PRMbase import *
from Maps  import *

''' change the following values to see how the algorithm works'''
map_size = (1000, 600)
num_objects = 200
init = (50,100)
goal = (850,500)




menu = ['''
PRM PLANNER DEMOS - Miguel Hernando
Press any key to start:
0 - Display Menu
1 - PRM simple [default]
2 - RRT-Connect 
3 - RRT*

m - Load a Map 
'''        
if __name__ == '__main__':
    pygame.init()
    map = BaseMap(*map_size) 
    map.loadMap(map1, [init,goal])
    
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
