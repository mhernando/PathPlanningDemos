import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin

# Comparativa de sistemas de muestreo en una n-esfera


def gen_100_puntos(n):
    puntos = []
    for i in range(n):
        r = np.random.uniform(0,1)
        th = np.random.uniform(0,2*np.pi)
        puntos.append((r*cos(th), r*sin(th)))
    return puntos
def gen_100_puntos_sqrt(n):
    puntos = []
    for i in range(n):
        r = np.random.uniform(0,1)**0.5
        th = np.random.uniform(0,2*np.pi)
        puntos.append((r*cos(th), r*sin(th)))
    return puntos

def gen_100_puntos_filtro(n):
    puntos = []
    while len(puntos) < n:
        x, y = np.random.uniform(-1, 1, 2)
        if x**2 + y**2 <= 1:
            puntos.append((x, y))
    return puntos

def plot_puntos(fig, titulo, puntos):
    # Graficar los puntos
    #fig.figure(figsize=(6, 6))
    fig.scatter(puntos[:, 0], puntos[:, 1], color='blue', s=2)
    fig.set_aspect('equal', adjustable='box')
    fig.set_title(titulo)
    fig.set_xlabel('X')
    fig.set_ylabel('Y')
    fig.grid(True)
    
# Convertir la lista de puntos a un array de numpy
generadores=((gen_100_puntos ,"R=random(0,1), th=random(0,2pi)"),
 (gen_100_puntos_sqrt , "R=sqrt(random(0,1), th=random(0,2pi)"),
 (gen_100_puntos_filtro , "x,y=random(-1,1) filtrado con R=1"))
num_puntos = 1000
size=4
fig, axs = plt.subplots(1, len(generadores), figsize=(size*len(generadores),size))

for i in range(len(generadores)):
    puntos = np.array(generadores[i][0](num_puntos))
    plot_puntos(axs[i],generadores[i][1], puntos)
plt.suptitle('Distribuciones de {0} puntos en un círculo unitario'.format(num_puntos))
plt.tight_layout()
plt.show()