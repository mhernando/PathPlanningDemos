import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from logger import *
import os
class ExperimentManager:
    def __init__(self):
        self.reset()

    def start_experiment(self, exp_id):
        self.current_exp_id = exp_id
        self.experiments[exp_id] = DataLogger()
    
    def reset(self):
        """Elimina todos los experimentos y reinicia el estado."""
        self.experiments = {}
        self.current_exp_id = None
        self.normalized_data = None
        self.normalized_data_t = None
        self.normalized_time = None

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
    def get_experiments_time(self):
        time = 0
        for exp_id, logger in self.experiments.items():
            time+=logger.time[-1]
        return time
    def save_all(self, file_name):
        lenght_data = {}
        cost_data = {}
        time_data = {}

        for exp_id, logger in self.experiments.items():
            lenght_data[f'Exp_{exp_id}'] = logger.lenght
            cost_data[f'Exp_{exp_id}'] = logger.cost
            time_data[f'Exp_{exp_id}'] = logger.time

        df_lenght = pd.DataFrame(dict(lenght_data))
        df_cost = pd.DataFrame(dict(cost_data))
        df_time = pd.DataFrame(dict(time_data))
        data=self.normaliza_datos()
      
        df_data = pd.DataFrame(list(zip(*data)))
        df_data.columns = [f'Exp {i+1}' for i in range(len(data))]

        with pd.ExcelWriter(file_name, engine='openpyxl') as writer:
            df_lenght.to_excel(writer, sheet_name='Lenght', index=False)
            df_cost.to_excel(writer, sheet_name='Cost', index=False)
            df_time.to_excel(writer, sheet_name='Time', index=False)
            df_data.to_excel(writer, sheet_name='L_Normalized', index=False)
        
        datos_longitud_normalizados, datos_tiempo_normalizados = self.normaliza_datos_con_tiempo()

        df_l_norm = pd.DataFrame(datos_longitud_normalizados).transpose()
        df_t_norm = pd.DataFrame(datos_tiempo_normalizados).transpose()

        df_l_norm.columns = [f'Exp {i+1}' for i in range(len(datos_longitud_normalizados))]
        df_t_norm.columns = [f'Exp {i+1}' for i in range(len(datos_tiempo_normalizados))]

    def save_remuestreado(self, file_name, valor_optimo_esperado):
        datos, tiempos = self.normaliza_datos_con_tiempo()
        tiempo_max = max(t[-1] for t in tiempos)
        base_tiempos = np.linspace(0, tiempo_max, 200)

        remuestreados = [np.interp(base_tiempos, t, v) for v, t in zip(datos, tiempos)]

        df_remuestreo = pd.DataFrame(remuestreados).T
        df_remuestreo.insert(0, 'Tiempo', base_tiempos)

        with pd.ExcelWriter(file_name, engine='openpyxl', mode='a', if_sheet_exists='replace') as writer:
            df_remuestreo.to_excel(writer, sheet_name='Remuestreo', index=False)

        matriz = np.array(remuestreados)
        media = np.mean(matriz, axis=0)
        desviacion = np.std(matriz, axis=0)
        
        df_estadisticas = pd.DataFrame({'Time': base_tiempos,'Media': media, 'Desviacion': desviacion})

        base_name, ext = os.path.splitext(file_name)
        estadisticas_file = base_name + '_m_desv.xlsx'
        df_estadisticas.to_excel(estadisticas_file, index=False)

        resumen_file = base_name + '.txt'
        with open(resumen_file, 'w') as f:
            valor_inicial_medio = media[0]
            desviacion_inicial = desviacion[0]
            valor_final_medio = media[-1]
            desviacion_final = desviacion[-1]

            f.write(f'Valor inicial medio: {valor_inicial_medio:.4f}\n')
            f.write(f'Valor inicial medio y desviación: {valor_inicial_medio:.4f}, {desviacion_inicial:.4f}\n')
            f.write(f'Valor final medio y desviación: {valor_final_medio:.4f}, {desviacion_final:.4f}\n')
            if valor_optimo_esperado:                 
                u50 = valor_optimo_esperado + 0.50 * (valor_inicial_medio - valor_optimo_esperado)
                u25 = valor_optimo_esperado + 0.25 * (valor_inicial_medio - valor_optimo_esperado)
                u10 = valor_optimo_esperado + 0.10 * (valor_inicial_medio - valor_optimo_esperado)
                f.write(f'Valor optimo: {valor_optimo_esperado:.4f}\n')
                self.tiempo_umbral(f, base_tiempos, media, u50, "50%")
                self.tiempo_umbral(f, base_tiempos, media, u25, "25%")
                self.tiempo_umbral(f, base_tiempos, media, u10, "10%")
                '''f.write(f'Valor optimo: {valor_optimo_esperado:.4f}, umbral(50%): {u50}, umbral(25%): {u25},umbral(10%): {u10},\n')
                indices_50 = np.where(media <= u50)[0]
                tiempo_50 = base_tiempos[indices_50[0]] if len(indices_50) > 0 else None
                if tiempo_50: f.write(f'Tiempo 50% valor óptimo = {tiempo_50}\n')
                else: f.write(f'No consige el 50% del valor óptimo \n')
                '''

                
    def tiempo_umbral(self, file, tiempos, medias, umbral, txt):
        indices = np.where(medias <= umbral)[0]
        tiempo = tiempos[indices[0]] if len(indices) > 0 else None
        if tiempo: file.write(f'Tiempo {txt} valor óptimo = {tiempo}\n')
        else: file.write(f'No consige el {txt} del valor óptimo \n')
        
    def normaliza_datos(self):
        #elimina los ceros iniciales y rellena al final y lo retorna todo como una matriz
        if self.normalized_data: return self.normalized_data
        recortados=[]
        for exp_id, logger in self.experiments.items():
            v=logger.lenght
            t=logger.time
            i=0       
            while i < len(v) and v[i] == 0 : i += 1
            v2=v[i:]
            if not v2: continue
            #t=[0]+t
            recortados.append(v2)
        max_len = max(len(v2) for v2 in recortados) if recortados else  0 
        salida = []
        for v2 in recortados:
            fill = v2[-1]
            salida.append(v2 + [fill] * (max_len - len(v2)))
        self.normalized_data =  salida
        return salida

    
    def normaliza_datos_con_tiempo(self):
        if self.normalized_data_t and self.normalized_time:
            return self.normalized_data_t, self.normalized_time

        recortados_lenght = []
        recortados_time = []

        for exp_id, logger in self.experiments.items():
            v = logger.lenght
            t = logger.time
            i = 0
            while i < len(v) and v[i] == 0:
                i += 1
            v2 = v[i:]
            t2 = t[i:]
            if not v2 or not t2:
                continue
            t0 = t2[0]
            t2 = [ti - t0 for ti in t2]
            recortados_lenght.append(v2)
            recortados_time.append(t2)

        max_time = max(t[-1] for t in recortados_time) if recortados_time else 0

        salida_lenght = []
        salida_time = []

        for v2, t2 in zip(recortados_lenght, recortados_time):
            fill = v2[-1]
            last_time = t2[-1]
            dt = t2[-1] - t2[-2] if len(t2) > 1 else 0.1
            while last_time < max_time:
                last_time += dt
                v2.append(fill)
                t2.append(last_time)
            salida_lenght.append(v2)
            salida_time.append(t2)

        self.normalized_data_t = salida_lenght
        self.normalized_time = salida_time
        return salida_lenght, salida_time

    def plot_normalizado_con_tiempo(self):
        data, tiempos = self.normaliza_datos_con_tiempo()
        datos = np.array(data)
        tiempos = np.array(tiempos)

        media = np.mean(datos, axis=0)
        desviacion = np.std(datos, axis=0)
        x = np.mean(tiempos, axis=0)

        plt.figure(figsize=(10, 5))
        plt.plot(x, media, label='Media', color='blue')
        plt.fill_between(x, media - desviacion, media + desviacion,
                         color='blue', alpha=0.3, label='±1 Desviación estándar')

        plt.title('Media y Desviación Estándar (Normalizado por Tiempo)')
        plt.xlabel('Tiempo')
        plt.ylabel('Longitud')
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.tight_layout()
        plt.show()

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

