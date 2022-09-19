from RRTdemos import *

context = {
    'event':False,
    'key':None,
    'planner': None,
    'map': None
    }

def key_press(key):
    c=key.char
    print("Key pressed:", c)
def sel():
    global context
    print(context['sel_planner'].get())
    
def init_gui_window():
    global context
    context['gui'] = root = tk.Tk()

    text = tk.Button(root, text='Blah.')
    text.pack()
    
    context['sel_planner'] = var = tk.IntVar()
    
    
    planners = [("Simple RRT", 101),
                 ("RRT-Connect", 102),
                 ("RRT-Star", 103),
                 ("RRT (Dubbin's car)", 104)]


    sel_frame = tk.Frame(root, borderwidth=1, relief=tk.RIDGE)
    
    for option, val in planners:
        tk.Radiobutton(sel_frame, 
                       text=option,
                       padx = 20, 
                       variable=var, 
                       command=sel,
                       value=val).pack(anchor=tk.W)
    sel_frame.pack(padx=5, pady=5)
    
    context['sel_2'] = var2 = tk.IntVar()
    sel_frame2 = tk.Frame(root,)
    for option, val in planners:
        tk.Radiobutton(sel_frame2, 
                       text=option,
                       padx = 20, 
                       variable=var2, 
                       command=sel,
                       value=val).pack(anchor=tk.W)
    sel_frame2.pack(padx=5, pady=5)
    
    root.bind('<KeyPress>',key_press)
    root.update()
    root.mainloop()
def init_defaults():
     global context
     context['map'] = BaseMap(*map_size)
     context['map'].loadMap(map1, [init,goal])
     context['planner'] = rrt_simple


if __name__ == '__main__':
 
    pygame.init()
    init_gui_window()
    init_defaults()
   
    map = context['map'] 
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
