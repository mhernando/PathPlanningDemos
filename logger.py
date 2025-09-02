import pandas as pd
import time

class DataLogger:
    def __init__(self):
        self.reset()

    def add_sample(self, iteration, lenght, cost):
        _time = self.get_elapsed_time()
        self.time.append(_time)
        self.iteration.append(iteration)
        self.lenght.append(lenght)
        self.cost.append(cost)
        print("Logger: ", iteration)

    def pause(self):
        if self._pause_start is None:
            self._pause_start = time.time()

    def resume(self):
        if self._pause_start is not None:
            self._paused_time += time.time() - self._pause_start
            self._pause_start = None

    def get_elapsed_time(self):
        now = time.time()
        if self._pause_start is not None:
            now = self._pause_start  # no avanza el tiempo durante la pausa
        return now - self._start_time - self._paused_time

    def reset(self):
        self.time = []
        self.iteration = []
        self.lenght = []
        self.cost = []
        self._start_time = time.time()
        self._paused_time = 0
        self._pause_start = None


    def save(self, file_name):
        df = pd.DataFrame({
            'Time': self.time,
            'Iteration': self.iteration,
            'Lenght': self.lenght,
            'Cost': self.cost
        })
        df.to_excel(file_name, index=False)