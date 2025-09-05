from RRTdemos import *
from RRTmhg import *
from logger import DataLogger
from experimentmanager import ExperimentManager
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
    'logger': DataLogger(),
    'experiments': ExperimentManager(),
    'exp_state': State.STOP,
    'exp_n':10,
    'exp_max_iter':1000,
    'exp_current':0
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
    if context['exp_state'] == State.PAUSE:
        context['experiments'].resume()
        context['exp_state'] = State.PLAY
    context['state'] = State.PLAY
    context['logger'].resume()
    update_UI_states()
def pause():
    global context
    context['state'] = State.PAUSE
    context['logger'].pause()
    if context['exp_state'] == State.PLAY:
        context['experiments'].pause()
        context['exp_state'] = State.PAUSE
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
def experiments():
    global context
    if context['exp_state'] != State.STOP: return
    dialog = tk.Toplevel(context['gui'])
    dialog.title("Configuración del experimento")

    tk.Label(dialog, text="Número máximo de muestras:").grid(row=0, column=0, padx=10, pady=5)
    entry_muestras = tk.Entry(dialog)
    entry_muestras.grid(row=0, column=1, padx=10, pady=5)

    tk.Label(dialog, text="Número de experimentos:").grid(row=1, column=0, padx=10, pady=5)
    entry_experimentos = tk.Entry(dialog)
    entry_experimentos.grid(row=1, column=1, padx=10, pady=5)

    entry_muestras.insert(0, str(context['exp_max_iter']))
    entry_experimentos.insert(0, str(context['exp_n']))
    def on_ok():
        try:
            num_muestras = int(entry_muestras.get())
            num_experimentos = int(entry_experimentos.get())
            dialog.destroy()
            iniciar_experimento(num_muestras, num_experimentos)
        except ValueError:
            messagebox.showerror("Error", "Por favor, introduce valores numéricos válidos.")

    def on_cancel():
        dialog.destroy()

    tk.Button(dialog, text="OK", command=on_ok).grid(row=2, column=0, padx=10, pady=10)
    tk.Button(dialog, text="Cancel", command=on_cancel).grid(row=2, column=1, padx=10, pady=10)
  
    
def stop():
    global context  
    context['state'] = State.STOP
    context['logger'].pause()
    #gestionar el stop de los experimentos
    context['exp_state'] = State.STOP
    update_UI_states()

def iniciar_experimento(num_muestras, num_experimentos):
    global context
    context['exp_n']=num_experimentos
    context['exp_max_iter']=num_muestras
    context['experiments'].reset()
    context['exp_current']=0
    context['experiments'].start_experiment(0)
    context['exp_state'] = State.PLAY
    
    play()
    
def end_experiments():
    context['exp_state'] = State.STOP
    total_time = context['experiments'].get_experiments_time()
    num=context['exp_current'] 
    print(f"TOTAL TIME: {total_time} NUM EXP:{num} TIME_PER_EXP:{total_time/num}")
     
    context['experiments'].plot_normalizado()
    file_name = filedialog.asksaveasfilename(
        defaultextension=".xlsx",
        filetypes=[("Excel files", "*.xlsx")],
        title="Save planning data as ..."
    ) 
    if file_name:
        context['experiments'].save_all(file_name)
    else:
        messagebox.showwarning("Cancelado", "No se seleccionó ningún archivo.")
    #context['experiments'].plot_all()
    print("EXPERIMENT END")
    stop()

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
    bplay, bpause, bstop , bexp= context['play'], context['pause'], context['stop'],  context['b_experiments']
    frm_state = "disabled"
    if state == State.STOP :
        bpause["state"]="disabled"
        bplay["state"]="normal"
        bstop["state"]="disabled"
        bexp["state"]="normal"
        frm_state="normal"
    if state == State.PAUSE :
        bpause["state"]="disabled"
        bplay["state"]="normal"
        bstop["state"]="normal"
        bexp["state"]="disabled"

    if state == State.PLAY :
        bpause["state"]="normal"
        bplay["state"]="disabled"
        bstop["state"]="normal"
        bexp["state"]="disabled"

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
    context['b_experiments']=bsave=tk.Button(root, text="EXPERIMENTS", command=experiments)
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
            iterate()

#main function responsible of iterations.
'''
    'experiments': ExperimentManager(),
    'exp_state': State.STOP
    'exp_n':10
    'exp_max_iter':1000
    'exp_current':0
'''
def iterate():
    
    logger = context['logger']
    planner = context['planner']
    #if there is a running experiment the iteration is controlled by it
    if context['exp_state']==State.PLAY:
        logger = context['experiments'].get_logger()
        if planner.iterations >= context['exp_max_iter']:
            context['state'] = State.STOP
            context['exp_current']=context['exp_current']+1
            ##new experiment if exp_current < exp_n
            if context['exp_current'] < context['exp_n'] :
                play()
                context['experiments'].start_experiment(context['exp_current'])
                logger = context['experiments'].get_logger()
            else: ##otherwise manage the end of the experiment
                end_experiments()
                return
    #normal execution
    if context['planner'].iterate(10, logger):pause()
       

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
