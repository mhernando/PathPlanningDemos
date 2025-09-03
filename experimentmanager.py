import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from logger import * 
class ExperimentManager:
    def __init__(self):
        self.experiments = {}
        self.current_exp_id = None

    def start_experiment(self, exp_id):
        self.current_exp_id = exp_id
        self.experiments[exp_id] = DataLogger()
    
    def reset(self):
        """Elimina todos los experimentos y reinicia el estado."""
        self.experiments = {}
        self.current_exp_id = None

    def pause(self):
        """Pausa el experimento actual."""
        if self.current_exp_id is not None:
            self.experiments[self.current_exp_id].pause()

    def resume(self):
        """Reanuda el experimento actual."""
        if self.current_exp_id is not None:
            self.experiments[self.current_exp_id].resume()

    def get_logger(self):
        if self.current_exp_id is None:
            raise ValueError("No experiment started. Use start_experiment(exp_id) first.")
        return self.experiments[self.current_exp_id]
    def add_sample(self, iteration, lenght, cost):
        if self.current_exp_id is None:
            raise ValueError("No experiment started. Use start_experiment(exp_id) first.")
        self.experiments[self.current_exp_id].add_sample(iteration, lenght, cost)

    def save_all(self, file_name):
        lenght_data = {}
        cost_data = {}

        for exp_id, logger in self.experiments.items():
            lenght_data[f'Exp_{exp_id}'] = logger.lenght
            cost_data[f'Exp_{exp_id}'] = logger.cost

        df_lenght = pd.DataFrame(dict(lenght_data))
        df_cost = pd.DataFrame(dict(cost_data))

        with pd.ExcelWriter(file_name, engine='openpyxl') as writer:
            df_lenght.to_excel(writer, sheet_name='Lenght', index=False)
            df_cost.to_excel(writer, sheet_name='Cost', index=False)

    def normaliza_datos(self):
        #elimina los ceros iniciales y rellena al final y lo retorna todo como una matriz
        recortados=[]
        for exp_id, logger in self.experiments.items():
            v=logger.lenght
            i=0       
            while i < len(v) and v[i] == 0 : i += 1
            t=[0]+v[i:]
            if not t: continue
            else: recortados.append(t)
        max_len = max(len(t) for t in recortados) if recortados else  0 
        salida = []
        for t in recortados:
            fill = t[-1]
            salida.append(t + [fill] * (max_len - len(t)))
        return salida

    def plot_normalizado(self):
        data=self.normaliza_datos()
        
        # Convertir a array para facilitar cálculos
        datos = np.array(data)  # shape: (n_experimentos, n_muestras)

        # Calcular media y desviación estándar por posición (columna)
        media = np.mean(datos, axis=0)
        desviacion = np.std(datos, axis=0)

        # Eje X (puede ser tiempo u otro índice)
        x = np.arange(datos.shape[1])

        # Crear gráfico
        plt.figure(figsize=(10, 5))
        plt.plot(x, media, label='Media', color='blue')
        plt.fill_between(x, media - desviacion, media + desviacion,
                         color='blue', alpha=0.3, label='±1 Desviación estándar')

        plt.title('Media y Desviación Estándar')
        plt.xlabel('Índice de muestra')
        plt.ylabel('Valor')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.show()
        
    def plot_all(self):
        plt.figure(figsize=(12, 5))

        # Plot Lenght
        plt.subplot(1, 2, 1)
        for exp_id, logger in self.experiments.items():
            plt.plot(logger.lenght, label=f'Exp {exp_id}')
        plt.title('Evolución de la Longitud')
        plt.xlabel('Número de muestra')
        plt.ylabel('Longitud')
        plt.legend()
        plt.grid(True)

        # Plot Cost
        plt.subplot(1, 2, 2)
        for exp_id, logger in self.experiments.items():
            plt.plot(logger.cost, label=f'Exp {exp_id}')
        plt.title('Evolución del Coste')
        plt.xlabel('Número de muestra')
        plt.ylabel('Coste')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.show()

