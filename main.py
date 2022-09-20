from RRTdemos import *
from enum import Enum
from threading import *
class State(Enum):
    PLAY = 1
    PAUSE = 2
    STOP = 3
    
context = {
    'event':False,
    'key':None,
    'planner': None,
    'map': None,
    'state': State.STOP,
    'end': False
    }
planners = [("(1) Simple RRT", 0, RRT),
            ("(2) RRT-Connect", 1, RRTconnect),
            ("(3) RRT-Star", 2, RRTstar),
            ("(4) RRT (Dubbin's car)", 3, RRTdubbins)]  
def key_press(key):
    c=key.char
    print("Key pressed:", c)
def sel():
    global context
    print(planners[context['sel_planner'].get()])
    
def play():
    global context
    if context['state'] != State.PAUSE:
        tree = context['sel_tree'].get()
        if tree == 0: set_tree_type(TreeBase)
        else: set_tree_type(TreeDiscretized)
        planner=planners[context['sel_planner'].get()][2]
        map = context['map']
        context['planner']=planner(map, init, goal)

        map.draw()
        map.draw_init_and_goal(init,goal)
    context['state'] = State.PLAY
    update_UI_states()
def pause():
    global context
    context['state'] = State.PAUSE
    update_UI_states()
def frame_state(frame, state):
    for child in frame.winfo_children():
       child.configure(state=state)
def stop():
    global context
    context['state'] = State.STOP
    update_UI_states()

def set_map(nmap):
    map=context['map']
    if nmap==1:
        map.createRandomMap(num_objects, [init,goal])  
    if nmap==2: 
        map.loadMap(map1, [init,goal])
    if nmap==3: 
        map.loadMap(map2, [init,goal])
    if nmap==4: 
        map.loadMap(map3, [init,goal])
    map.draw()
    map.draw_init_and_goal(init,goal)
    pygame.display.update()    
    print("Masp", map)
    update_UI_states()

def update_UI_states():
    global context
    state = context['state']
    bplay, bpause, bstop = context['play'], context['pause'], context['stop']
    frm_state = "disabled"
    if state == State.STOP :
        bpause["state"]="disabled"
        bplay["state"]="normal"
        bstop["state"]="disabled"
        frm_state="normal"
    if state == State.PAUSE :
        bpause["state"]="disabled"
        bplay["state"]="normal"
        bstop["state"]="normal"

    if state == State.PLAY :
        bpause["state"]="normal"
        bplay["state"]="disabled"
        bstop["state"]="normal"

    frame_state(context['frame_planners'],frm_state)
    frame_state(context['frame_trees'],frm_state)
    frame_state(context['frame_map'],frm_state)
def on_close():
    global context
    context['end']=True
    context['gui'].destroy()
def init_gui_window():
    global context
    context['gui'] = root = tk.Tk()
    root.protocol("WM_DELETE_WINDOW", on_close)

    ################################PLANNER TYPES########################   
    context['sel_planner'] = var = tk.IntVar()


    context['frame_planners']=sel_frame = tk.Frame(root, borderwidth=2, relief=tk.GROOVE)
    for option, val, plan in planners:
        tk.Radiobutton(sel_frame, 
                       text=option,
                       padx = 20, 
                       variable=var, 
                       command=sel,
                       value=val).pack(anchor=tk.W)
    sel_frame.pack(padx=5, pady=5)
    ################################## TREE TYPES############################
    
    context['sel_tree'] = var2 = tk.IntVar()
    context['frame_trees']=sel_frame2 = tk.Frame(root,borderwidth=2, relief=tk.GROOVE)
    tk.Radiobutton(sel_frame2, text="Continuous tree(1,2)",
                       padx = 20, variable=var2, 
                       command=sel, value=0).pack(anchor=tk.W)
    tk.Radiobutton(sel_frame2, text="Discretized tree",
                       padx = 20, variable=var2, 
                       command=sel, value=1).pack(anchor=tk.W)

    sel_frame2.pack(padx=5, pady=5)
    
    ##########################CONTROL FRAME##################################
    control_frame = tk.Frame(root)
    context['play']=bplay=tk.Button(control_frame, text="PLAY", command=play)
    bplay.pack(side=tk.LEFT, pady=15)
    context['pause']=bpause=tk.Button(control_frame, text="PAUSE", command=pause)
    bpause.pack(side=tk.LEFT)
    context['stop']=bstop=tk.Button(control_frame, text="STOP", command=stop)
    bstop.pack(side=tk.RIGHT)
    control_frame.pack(padx=5, pady=5)
    ##########################MAP FRAME##################################
    context["frame_map"]=frame_map = tk.Frame(root,borderwidth=2, relief=tk.GROOVE)
    tk.Button(frame_map, text="Load Random map", command=lambda: set_map(1)).pack(fill=tk.X)
    tk.Button(frame_map, text="Load Map 1", command=lambda: set_map(2)).pack(fill=tk.X)
    tk.Button(frame_map, text="Load Map 2", command=lambda: set_map(3)).pack(fill=tk.X)
    tk.Button(frame_map, text="Load unsolvable map", command=lambda: set_map(4)).pack(fill=tk.X)
    frame_map.pack(padx=5, pady=5)
    ######################################################################
    update_UI_states()
    root.bind('<KeyPress>',key_press)
    #root.update()
    #root.mainloop()
def init_defaults():
     global context
     map= context['map'] = BaseMap(*map_size)
     context['map'].loadMap(map1, [init,goal])
     context['planner'] = RRT(map,init, goal)

def control_loop():
    global context
    while(not context['end']):
        state = context['state']
        if state == State.PLAY:
            pygame.display.update()
            if context['planner'].iterate(10): pause()
if __name__ == '__main__':
 
    pygame.init()
    init_gui_window()
    init_defaults()
   
    map = context['map'] 
    map.draw()
    map.draw_init_and_goal(init,goal)
    
    pygame.display.update()
    
    Thread(target=control_loop).start() 
    context['gui'].mainloop()
    
    
    pygame.quit()
