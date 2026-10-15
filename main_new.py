from enum import Enum
from time import perf_counter

import pygame
import tkinter as tk

from tkinter import filedialog
from tkinter import messagebox

from RRTdemos import *
from RRTmhg import *
from logger import DataLogger
from experimentmanager import ExperimentManager


# ---------------------------------------------------------------------
# Application configuration
# ---------------------------------------------------------------------

TARGET_FPS = 30
FRAME_PERIOD_MS = round(1000 / TARGET_FPS)

# Maximum computation time assigned to the planner in every frame.
# At 60 FPS, 6-10 ms is a reasonable initial range.
#30 FPS con 20 ms es más rapido
COMPUTE_BUDGET_SECONDS = 0.02

# Additional protection for the initial stages, where an iteration can
# be extremely fast.
MAX_ITERATIONS_PER_FRAME = 200

# UI text does not need to be updated at 60 Hz.
UI_UPDATE_PERIOD_SECONDS = 0.10


class State(Enum):
    PLAY = 1
    PAUSE = 2
    STOP = 3


context = {
    "event": False,
    "key": None,
    "planner": None,
    "map": None,
    "state": State.STOP,
    "end": False,

    "logger": DataLogger(),
    "experiments": ExperimentManager(),

    "exp_state": State.STOP,
    "exp_n": 10,
    "exp_max_iter": 1000,
    "exp_current": 0,
    "exp_optimal": None,

    "last_ui_update": 0.0,
    "after_id": None,
}


planners = [
    ("(1) Simple RRT", 0, RRT),
    ("(2) RRT-Connect", 1, RRTconnect),
    ("(3) RRT-Star", 2, RRTstar),
    ("(4) Informed-RRT-Star", 3, informedRRTstar),
    ("(5) N-informed-RRT-Star", 4, n_informedRRTstar),
    ("(6) RRT (Dubbin's car)", 5, RRTdubbins),
]


# ---------------------------------------------------------------------
# Planner creation and configuration
# ---------------------------------------------------------------------

def configure_tree_type():
    """
    Configure the tree implementation selected in the GUI.
    """
    selected_tree = context["sel_tree"].get()

    if selected_tree == 0:
        set_tree_type(TreeBase)
    else:
        set_tree_type(TreeDiscretized)


def create_selected_planner():
    """
    Create a new planner using the currently selected algorithm and map.

    This function also redraws the basic map, but does not update the
    physical Pygame display. Display updates are centralized in the main
    application loop.
    """
    configure_tree_type()

    planner_index = context["sel_planner"].get()
    planner_class = planners[planner_index][2]

    map_ = context["map"]

    context["planner"] = planner_class(
        map_,
        init,
        goal,
    )

    map_.draw()
    map_.draw_init_and_goal(
        init,
        goal,
    )

    return context["planner"]


def planner_stops_at_first_solution(planner):
    """
    Determine whether a planner should stop after finding its first path.

    RRT and RRT-Connect normally stop at their first solution.
    RRT*, Informed RRT* and NIRRT* should continue optimizing.

    Prefer the explicit class attribute stop_at_first_solution. The
    fallback preserves compatibility with the current planner classes.
    """
    explicit_value = getattr(
        planner,
        "stop_at_first_solution",
        None,
    )

    if explicit_value is not None:
        return explicit_value

    return isinstance(
        planner,
        (
            RRT,
            RRTconnect,
            RRTdubbins,
        ),
    )


# ---------------------------------------------------------------------
# GUI callbacks
# ---------------------------------------------------------------------

def key_press(event):
    character = event.char

    if character:
        print("Key pressed:", character)


def planner_selection_changed():
    selected_index = context["sel_planner"].get()
    print("Selected planner:", planners[selected_index][0])


def play():
    """
    Start a new execution or resume a paused execution.
    """
    previous_state = context["state"]

    if previous_state == State.PAUSE:
        context["state"] = State.PLAY
        context["logger"].resume()

        if context["exp_state"] == State.PAUSE:
            context["experiments"].resume()
            context["exp_state"] = State.PLAY

        update_UI_states()
        return

    # A new manual execution is requested.
    if context["exp_state"] == State.STOP:
        context["logger"].reset()
        context["logger"].resume()

    create_selected_planner()

    context["state"] = State.PLAY

    update_UI_states()


def pause():
    """
    Pause the planner and, if applicable, the experiment timer.
    """
    if context["state"] != State.PLAY:
        return

    context["state"] = State.PAUSE
    context["logger"].pause()

    if context["exp_state"] == State.PLAY:
        context["experiments"].pause()
        context["exp_state"] = State.PAUSE

    update_UI_states()


def stop():
    """
    Stop the currently running planner or experiment batch.
    """
    context["state"] = State.STOP
    context["logger"].pause()

    if context["exp_state"] in (
        State.PLAY,
        State.PAUSE,
    ):
        try:
            context["experiments"].pause()
        except AttributeError:
            pass

    context["exp_state"] = State.STOP

    update_UI_states()


def frame_state(frame, state):
    """
    Enable or disable every direct widget contained in a frame.
    """
    for child in frame.winfo_children():
        try:
            child.configure(state=state)
        except tk.TclError:
            # Some widget types may not expose a state property.
            pass


def save():
    """
    Save the manual-execution logger.
    """
    if context["state"] not in (
        State.STOP,
        State.PAUSE,
    ):
        return

    context["logger"].pause()

    file_name = filedialog.asksaveasfilename(
        defaultextension=".xlsx",
        filetypes=[
            ("Excel files", "*.xlsx"),
        ],
        title="Save planning data as ...",
    )

    if file_name:
        try:
            context["logger"].save(file_name)
        except Exception as error:
            messagebox.showerror(
                "Error",
                f"No se pudieron guardar los datos:\n{error}",
            )
    else:
        messagebox.showwarning(
            "Cancelado",
            "No se seleccionó ningún archivo.",
        )


# ---------------------------------------------------------------------
# Experiment handling
# ---------------------------------------------------------------------

def experiments():
    """
    Open the experiment configuration dialog.
    """
    if context["exp_state"] != State.STOP:
        return

    dialog = tk.Toplevel(context["gui"])
    dialog.title("Configuración del experimento")
    dialog.transient(context["gui"])
    dialog.grab_set()

    tk.Label(
        dialog,
        text="Número máximo de muestras:",
    ).grid(
        row=0,
        column=0,
        padx=10,
        pady=5,
        sticky="e",
    )

    entry_samples = tk.Entry(dialog)

    entry_samples.grid(
        row=0,
        column=1,
        padx=10,
        pady=5,
    )

    tk.Label(
        dialog,
        text="Número de experimentos:",
    ).grid(
        row=1,
        column=0,
        padx=10,
        pady=5,
        sticky="e",
    )

    entry_experiments = tk.Entry(dialog)

    entry_experiments.grid(
        row=1,
        column=1,
        padx=10,
        pady=5,
    )

    tk.Label(
        dialog,
        text="Óptimo:",
    ).grid(
        row=2,
        column=0,
        padx=10,
        pady=5,
        sticky="e",
    )

    entry_optimum = tk.Entry(dialog)

    entry_optimum.grid(
        row=2,
        column=1,
        padx=10,
        pady=5,
    )

    entry_samples.insert(
        0,
        str(context["exp_max_iter"]),
    )

    entry_experiments.insert(
        0,
        str(context["exp_n"]),
    )

    if context["exp_optimal"] is not None:
        entry_optimum.insert(
            0,
            str(context["exp_optimal"]),
        )

    def on_ok():
        try:
            number_of_samples = int(
                entry_samples.get()
            )

            number_of_experiments = int(
                entry_experiments.get()
            )

            optimum_text = entry_optimum.get().strip()

            if optimum_text:
                optimum = float(optimum_text)
            else:
                optimum = None

            if number_of_samples <= 0:
                raise ValueError(
                    "El número de muestras debe ser positivo."
                )

            if number_of_experiments <= 0:
                raise ValueError(
                    "El número de experimentos debe ser positivo."
                )

            if optimum is not None and optimum <= 0.0:
                raise ValueError(
                    "El óptimo debe ser positivo."
                )

            context["exp_optimal"] = optimum

            dialog.grab_release()
            dialog.destroy()

            start_experiments(
                number_of_samples,
                number_of_experiments,
            )

        except ValueError as error:
            messagebox.showerror(
                "Error",
                str(error),
                parent=dialog,
            )

    def on_cancel():
        dialog.grab_release()
        dialog.destroy()

    tk.Button(
        dialog,
        text="OK",
        command=on_ok,
    ).grid(
        row=3,
        column=0,
        padx=10,
        pady=10,
    )

    tk.Button(
        dialog,
        text="Cancel",
        command=on_cancel,
    ).grid(
        row=3,
        column=1,
        padx=10,
        pady=10,
    )

    dialog.protocol(
        "WM_DELETE_WINDOW",
        on_cancel,
    )

    entry_samples.focus_set()


def start_experiments(
    number_of_samples,
    number_of_experiments,
):
    """
    Initialize a complete experiment batch.
    """
    context["exp_n"] = number_of_experiments
    context["exp_max_iter"] = number_of_samples
    context["exp_current"] = 0

    context["experiments"].reset()

    start_experiment_run(0)


def start_experiment_run(experiment_index):
    """
    Prepare one independent run of the selected planner.
    """
    context["exp_current"] = experiment_index

    create_selected_planner()

    context["experiments"].start_experiment(
        experiment_index
    )

    context["exp_state"] = State.PLAY
    context["state"] = State.PLAY

    update_UI_states()


def current_experiment_finished():
    """
    Check whether the current run has reached its iteration limit.
    """
    if context["exp_state"] != State.PLAY:
        return False

    planner = context["planner"]

    if planner is None:
        return False

    return (
        planner.iterations
        >= context["exp_max_iter"]
    )


def advance_experiment():
    """
    Finish the current run and start the next one.

    Returns True when the caller must stop performing planner iterations
    in the current application frame.
    """
    next_experiment = context["exp_current"] + 1

    if next_experiment < context["exp_n"]:
        start_experiment_run(next_experiment)
        return True

    end_experiments()
    return True


def end_experiments():
    """
    Finalize, plot and optionally save an experiment batch.

    This function now runs in the Tkinter main thread, so file dialogs
    and message boxes are safe.
    """
    context["state"] = State.STOP
    context["exp_state"] = State.STOP

    try:
        context["experiments"].pause()
    except AttributeError:
        pass

    total_time = (
        context["experiments"].get_experiments_time()
    )

    completed_experiments = context["exp_n"]

    if completed_experiments > 0:
        average_time = (
            total_time / completed_experiments
        )
    else:
        average_time = 0.0

    print(
        f"TOTAL TIME: {total_time:.3f} "
        f"NUM EXP: {completed_experiments} "
        f"TIME PER EXP: {average_time:.3f}"
    )

    try:
        context["experiments"].plot_normalizado()
    except Exception as error:
        messagebox.showwarning(
            "Aviso",
            "No se pudo generar la gráfica normalizada:\n"
            f"{error}",
        )

    file_name = filedialog.asksaveasfilename(
        defaultextension=".xlsx",
        filetypes=[
            ("Excel files", "*.xlsx"),
        ],
        title="Save planning data as ...",
    )

    if file_name:
        try:
            context["experiments"].save_all(
                file_name
            )

            if context["exp_optimal"] is not None:
                context[
                    "experiments"
                ].save_remuestreado(
                    file_name,
                    context["exp_optimal"],
                )

        except Exception as error:
            messagebox.showerror(
                "Error",
                "No se pudieron guardar los experimentos:\n"
                f"{error}",
            )
    else:
        messagebox.showwarning(
            "Cancelado",
            "No se seleccionó ningún archivo.",
        )

    print("EXPERIMENT END")

    update_UI_states()


# ---------------------------------------------------------------------
# Map handling
# ---------------------------------------------------------------------

def set_map(index):
    """
    Load one predefined map or create a random one.
    """
    global init, goal

    if context["state"] != State.STOP:
        return

    map_ = context["map"]

    if index == -1:
        map_.createRandomMap(
            num_objects,
            [init, goal],
        )
    else:
        init = maps[index].init
        goal = maps[index].goal

        map_.loadMap(
            maps[index].map,
            [init, goal],
        )

    map_.draw()
    map_.draw_init_and_goal(
        init,
        goal,
    )

    pygame.display.flip()

    print("Map:", map_)

    update_UI_states()


def load_map():
    """
    Load a map from a .map file.
    """
    global init, goal

    if context["state"] != State.STOP:
        return

    map_ = context["map"]

    if map_ is None:
        return

    file_name = filedialog.askopenfilename(
        defaultextension=".map",
        filetypes=[
            ("Map files", "*.map"),
        ],
        title="Cargar mapa desde archivo...",
    )

    if not file_name:
        messagebox.showwarning(
            "Cancelado",
            "No se seleccionó ningún archivo.",
        )
        return

    try:
        init, goal = map_.load_map_from_file(
            file_name
        )

        map_.draw()
        map_.draw_init_and_goal(
            init,
            goal,
        )

        pygame.display.flip()

        messagebox.showinfo(
            "Mapa cargado",
            "Mapa cargado correctamente desde:\n"
            f"{file_name}",
        )

    except Exception as error:
        messagebox.showerror(
            "Error",
            "No se pudo cargar el mapa:\n"
            f"{error}",
        )


def save_map():
    """
    Save the current map into a .map file.
    """
    if context["state"] != State.STOP:
        return

    map_ = context["map"]

    if map_ is None:
        return

    file_name = filedialog.asksaveasfilename(
        defaultextension=".map",
        filetypes=[
            ("Map files", "*.map"),
        ],
        title="Save map data as ...",
    )

    if file_name:
        try:
            map_.save_map_to_file(
                file_name,
                init,
                goal,
            )
        except Exception as error:
            messagebox.showerror(
                "Error",
                "No se pudo guardar el mapa:\n"
                f"{error}",
            )
    else:
        messagebox.showwarning(
            "Cancelado",
            "No se seleccionó ningún archivo.",
        )


# ---------------------------------------------------------------------
# GUI state and initialization
# ---------------------------------------------------------------------

def update_UI_states():
    state = context["state"]

    play_button = context["play"]
    pause_button = context["pause"]
    stop_button = context["stop"]
    experiment_button = context["b_experiments"]

    controls_state = "disabled"

    if state == State.STOP:
        pause_button["state"] = "disabled"
        play_button["state"] = "normal"
        stop_button["state"] = "disabled"
        experiment_button["state"] = "normal"

        controls_state = "normal"

    elif state == State.PAUSE:
        pause_button["state"] = "disabled"
        play_button["state"] = "normal"
        stop_button["state"] = "normal"
        experiment_button["state"] = "disabled"

    elif state == State.PLAY:
        pause_button["state"] = "normal"
        play_button["state"] = "disabled"
        stop_button["state"] = "normal"
        experiment_button["state"] = "disabled"

    frame_state(
        context["frame_planners"],
        controls_state,
    )

    frame_state(
        context["frame_trees"],
        controls_state,
    )

    frame_state(
        context["frame_map"],
        controls_state,
    )

    context["save_map"]["state"] = (
        "normal"
        if state == State.STOP
        else "disabled"
    )

    context["load_map"]["state"] = (
        "normal"
        if state == State.STOP
        else "disabled"
    )


def on_close():
    """
    Terminate the cooperative loop and close both GUI systems.
    """
    if context["end"]:
        return

    context["end"] = True
    context["state"] = State.STOP
    context["exp_state"] = State.STOP

    after_id = context.get("after_id")

    if after_id is not None:
        try:
            context["gui"].after_cancel(
                after_id
            )
        except tk.TclError:
            pass

    try:
        pygame.quit()
    except pygame.error:
        pass

    try:
        context["gui"].destroy()
    except tk.TclError:
        pass


def init_gui_window():
    context["gui"] = root = tk.Tk()

    root.title("RRT planner demonstrations")
    root.protocol(
        "WM_DELETE_WINDOW",
        on_close,
    )

    # -----------------------------------------------------------------
    # Planner types
    # -----------------------------------------------------------------

    context["sel_planner"] = planner_variable = (
        tk.IntVar(value=0)
    )

    context["frame_planners"] = planner_frame = (
        tk.Frame(
            root,
            borderwidth=2,
            relief=tk.GROOVE,
        )
    )

    for option, value, planner_class in planners:
        tk.Radiobutton(
            planner_frame,
            text=option,
            padx=20,
            variable=planner_variable,
            command=planner_selection_changed,
            value=value,
        ).pack(
            anchor=tk.W
        )

    planner_frame.pack(
        padx=5,
        pady=5,
        fill=tk.X,
    )

    # -----------------------------------------------------------------
    # Tree types
    # -----------------------------------------------------------------

    context["sel_tree"] = tree_variable = (
        tk.IntVar(value=0)
    )

    context["frame_trees"] = tree_frame = (
        tk.Frame(
            root,
            borderwidth=2,
            relief=tk.GROOVE,
        )
    )

    tk.Radiobutton(
        tree_frame,
        text="Continuous tree (RRT and RRT-Connect)",
        padx=20,
        variable=tree_variable,
        value=0,
    ).pack(
        anchor=tk.W
    )

    tk.Radiobutton(
        tree_frame,
        text="Discretized tree",
        padx=20,
        variable=tree_variable,
        value=1,
    ).pack(
        anchor=tk.W
    )

    tree_frame.pack(
        padx=5,
        pady=5,
        fill=tk.X,
    )

    # -----------------------------------------------------------------
    # Control frame
    # -----------------------------------------------------------------

    control_frame = tk.Frame(root)

    context["play"] = play_button = tk.Button(
        control_frame,
        text="PLAY",
        command=play,
    )

    play_button.pack(
        side=tk.LEFT,
        padx=4,
        pady=15,
    )

    context["pause"] = pause_button = tk.Button(
        control_frame,
        text="PAUSE",
        command=pause,
    )

    pause_button.pack(
        side=tk.LEFT,
        padx=4,
    )

    context["stop"] = stop_button = tk.Button(
        control_frame,
        text="STOP",
        command=stop,
    )

    stop_button.pack(
        side=tk.LEFT,
        padx=4,
    )

    control_frame.pack(
        padx=5,
        pady=5,
    )

    # -----------------------------------------------------------------
    # Status output
    # -----------------------------------------------------------------

    context["output"] = output_variable = (
        tk.StringVar(value="...")
    )

    output_label = tk.Label(
        root,
        textvariable=output_variable,
        font=("Arial", 8),
        fg="blue",
        bd=1,
        relief="sunken",
        anchor="w",
        justify="left",
        wraplength=300,
    )

    output_label.pack(
        fill=tk.X,
        padx=10,
        pady=10,
        ipady=10,
    )

    # -----------------------------------------------------------------
    # Save and experiment buttons
    # -----------------------------------------------------------------

    context["save"] = save_button = tk.Button(
        root,
        text="SAVE",
        command=save,
    )

    save_button.pack(
        padx=10,
        fill=tk.X,
    )

    context["b_experiments"] = experiment_button = (
        tk.Button(
            root,
            text="EXPERIMENTS",
            command=experiments,
        )
    )

    experiment_button.pack(
        padx=10,
        fill=tk.X,
    )

    # -----------------------------------------------------------------
    # Map frame
    # -----------------------------------------------------------------

    context["frame_map"] = map_frame = tk.Frame(
        root,
        borderwidth=2,
        relief=tk.GROOVE,
    )

    tk.Button(
        map_frame,
        text="Load Random map",
        command=lambda: set_map(-1),
    ).pack(
        fill=tk.X
    )

    for index, map_description in enumerate(maps):
        tk.Button(
            map_frame,
            text=map_description.description,
            command=lambda i=index: set_map(i),
        ).pack(
            fill=tk.X
        )

    map_frame.pack(
        padx=5,
        pady=5,
        fill=tk.X,
    )

    # -----------------------------------------------------------------
    # Map save/load
    # -----------------------------------------------------------------

    context["save_map"] = save_map_button = tk.Button(
        root,
        text="SAVE MAP",
        command=save_map,
    )

    save_map_button.pack(
        padx=10,
        fill=tk.X,
    )

    context["load_map"] = load_map_button = tk.Button(
        root,
        text="LOAD MAP",
        command=load_map,
    )

    load_map_button.pack(
        padx=10,
        fill=tk.X,
    )

    update_UI_states()

    root.bind(
        "<KeyPress>",
        key_press,
    )


def init_defaults():
    """
    Create the initial map.

    The planner is not created here because it should be created only
    when PLAY is pressed.
    """
    map_ = BaseMap(*map_size)

    context["map"] = map_

    map_.loadMap(
        map1,
        [init, goal],
    )


# ---------------------------------------------------------------------
# Pygame event processing
# ---------------------------------------------------------------------

def process_pygame_events():
    """
    Process all pending Pygame events from the main thread.
    """
    global init, goal

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            on_close()
            return

        if (
            context["state"] == State.STOP
            and event.type == pygame.MOUSEBUTTONDOWN
        ):
            if event.button == 1:
                init = event.pos
                print("INIT:", init)

            elif event.button == 3:
                goal = event.pos
                print("GOAL:", goal)

            else:
                continue

            map_ = context["map"]

            map_.draw()
            map_.draw_init_and_goal(
                init,
                goal,
            )


# ---------------------------------------------------------------------
# Logger and output
# ---------------------------------------------------------------------

def get_active_logger():
    if context["exp_state"] in (
        State.PLAY,
        State.PAUSE,
    ):
        return context["experiments"].get_logger()

    return context["logger"]


def update_output_text(force=False):
    """
    Update Tkinter and Pygame status text at a limited frequency.
    """
    current_time = perf_counter()

    if (
        not force
        and current_time - context["last_ui_update"]
        < UI_UPDATE_PERIOD_SECONDS
    ):
        return

    context["last_ui_update"] = current_time

    logger = get_active_logger()

    prefix = ""

    if context["exp_state"] in (
        State.PLAY,
        State.PAUSE,
    ):
        optimum = context["exp_optimal"]

        if optimum is None:
            optimum_text = "-"
        else:
            optimum_text = f"{optimum:.3f}"

        prefix = (
            f"EXP: {context['exp_current'] + 1}/"
            f"{context['exp_n']}  "
            f"Max-N: {context['exp_max_iter']}  "
            f"Opt: {optimum_text}\n"
        )

    try:
        iteration, elapsed_time, path_length = (
            logger.get_last_sample()
        )
    except (TypeError, ValueError):
        iteration = 0
        elapsed_time = 0.0
        path_length = 0.0

    status_line = (
        f"N: {iteration}  "
        f"T: {elapsed_time:.2f} s  "
        f"L: {path_length:.2f}"
    )

    context["output"].set(
        prefix + status_line
    )

    map_ = context["map"]

    if map_ is not None:
        map_.show_text(status_line)


# ---------------------------------------------------------------------
# Planner execution
# ---------------------------------------------------------------------

def execute_one_planner_iteration():
    """
    Execute one iteration of the active planner.

    Returns True if execution should stop for the current frame.
    """
    planner = context["planner"]

    if planner is None:
        return True

    logger = get_active_logger()

    solution_found = planner.iterate(
        1,
        logger,
    )

    # During experiments the iteration limit controls termination.
    # A first solution does not terminate an experiment run.
    if context["exp_state"] == State.PLAY:
        if current_experiment_finished():
            return advance_experiment()

        return False

    # Manual execution can stop at the first solution for non-optimizing
    # planners such as RRT and RRT-Connect.
    if (
        solution_found
        and planner_stops_at_first_solution(planner)
    ):
        stop()
        return True

    return False


def run_planner_for_budget(
    time_budget_seconds,
    maximum_iterations,
):
    """
    Execute planner iterations while respecting a real-time budget.

    The number of iterations per frame adapts automatically to the
    current computational cost of the tree.
    """
    deadline = perf_counter() + time_budget_seconds
    executed_iterations = 0

    while (
        context["state"] == State.PLAY
        and not context["end"]
        and executed_iterations < maximum_iterations
        and perf_counter() < deadline
    ):
        stop_current_frame = (
            execute_one_planner_iteration()
        )

        executed_iterations += 1

        if stop_current_frame:
            break


def render_planner_if_needed():
    """
    Render a planner that implements deferred rendering.

    Existing planner implementations that draw directly remain
    compatible. Refactored versions can expose render().
    """
    planner = context.get("planner")

    if planner is None:
        return

    render_method = getattr(
        planner,
        "render",
        None,
    )

    if callable(render_method):
        render_method()


# ---------------------------------------------------------------------
# Cooperative main loop
# ---------------------------------------------------------------------

def application_tick():
    """
    Main application frame.

    All Tkinter operations, Pygame event processing, Pygame rendering
    and planner execution occur in the same thread.
    """
    if context["end"]:
        return

    frame_start = perf_counter()

    process_pygame_events()

    if context["end"]:
        return

    if context["state"] == State.PLAY:
        run_planner_for_budget(
            COMPUTE_BUDGET_SECONDS,
            MAX_ITERATIONS_PER_FRAME,
        )

    render_planner_if_needed()
    update_output_text()

    try:
        pygame.display.flip()
    except pygame.error:
        on_close()
        return

    elapsed_seconds = (
        perf_counter() - frame_start
    )

    elapsed_milliseconds = (
        elapsed_seconds * 1000.0
    )

    delay_milliseconds = max(
        1,
        round(
            FRAME_PERIOD_MS
            - elapsed_milliseconds
        ),
    )

    try:
        context["after_id"] = (
            context["gui"].after(
                delay_milliseconds,
                application_tick,
            )
        )
    except tk.TclError:
        context["end"] = True


# ---------------------------------------------------------------------
# Program entry point
# ---------------------------------------------------------------------

if __name__ == "__main__":
    pygame.init()

    init_gui_window()
    init_defaults()

    map_ = context["map"]

    map_.draw()
    map_.draw_init_and_goal(
        init,
        goal,
    )

    pygame.display.flip()

    # Cooperative loop. No secondary thread is created.
    context["after_id"] = context["gui"].after(
        0,
        application_tick,
    )

    context["gui"].mainloop()

    if not context["end"]:
        context["end"] = True

    pygame.quit()