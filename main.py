from RRTdemos import *
from RRTmhg import *
from logger import DataLogger
from enum import Enum
from threading import *
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox

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
    'end': False,
    'logger': DataLogger()
    }
planners = [("(1) Simple RRT", 0, RRT),
            ("(2) RRT-Connect", 1, RRTconnect),
            ("(3) RRT-Star", 2, RRTstar),
            ("(4) Informed-RRT-Star",3,informedRRTstar),
            ("(5) N-informed-RRT-Star", 4, n_informedRRTstar),
            ("(6) RRT (Dubbin's car)", 5, RRTdubbins)]  
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
        context['logger'].reset()

        map.draw()
        map.draw_init_and_goal(init,goal)
    context['state'] = State.PLAY
    context['logger'].resume()
    update_UI_states()
def pause():
    global context
    context['state'] = State.PAUSE
    context['logger'].pause()
    update_UI_states()
def frame_state(frame, state):
    for child in frame.winfo_children():
       child.configure(state=state)
def save():
    global context
    if context['state'] not in (State.STOP, State.PAUSE):return
    context['logger'].pause()
    file_name = filedialog.asksaveasfilename(
        defaultextension=".xlsx",
        filetypes=[("Excel files", "*.xlsx")],
        title="Save planning data as ..."
    ) 
    if file_name:
        context['logger'].save(file_name)
    else:
        messagebox.showwarning("Cancelado", "No se seleccionó ningún archivo.")

    
def stop():
    global context  
    context['state'] = State.STOP
    context['logger'].pause()
    update_UI_states()

def set_map(i):
    map=context['map']
    global init, goal
    if i==-1:
        map.createRandomMap(num_objects, [init,goal])  
    else:
        init = maps[i].init
        goal = maps[i].goal
        map.loadMap(maps[i].map,[init, goal])
    '''if nmap==3: 
        map.loadMap(map2, [init,goal])
    if nmap==4: 
        map.loadMap(map3, [init,goal])
    if nmap==5: 
        map.loadMap(map4, [init,goal])
    '''
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
    
    context['save']=bsave=tk.Button(root, text="SAVE", command=save)
    bsave.pack(padx=10,fill=tk.X)
    ##########################MAP FRAME##################################
    context["frame_map"]=frame_map = tk.Frame(root,borderwidth=2, relief=tk.GROOVE)
    tk.Button(frame_map, text="Load Random map", command=lambda: set_map(-1)).pack(fill=tk.X)
    for i in range(len(maps)):
        tk.Button(frame_map, text=maps[i].description, command=lambda i=i: set_map(i)).pack(fill=tk.X)

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
def process_pygame_events():
    ev = pygame.event.get()
    for event in ev:
        if context['state']==State.STOP:
            global init, goal
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: #left
                    init = pygame.mouse.get_pos()
                if event.button == 3: #right
                    goal = pygame.mouse.get_pos()
                context['map'].draw()
                context['map'].draw_init_and_goal(init, goal)
                pygame.display.update()

def control_loop():
    global context
    while(not context['end']):
        process_pygame_events()
        state = context['state']
        if state == State.PLAY:
            pygame.display.update()
            if context['planner'].iterate(10, context['logger']):
                pause()
                context['logger'].pause()
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
