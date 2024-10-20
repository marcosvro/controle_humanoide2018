import numpy as np
import matplotlib.pyplot as plt
import time

# Definindo o vetor inicial
vector = np.array([4, 3])

# Função para calcular o vetor escalado
def scaled_vector(t, vector):
    scale = np.abs(np.sin(t))
    return scale * vector

# Configurações do gráfico
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal', 'box')

# Loop infinito para atualização contínua do gráfico
while True:
    t = time.time()  # Tempo atual em segundos
    
    scaled_vec = scaled_vector(t, vector)
    
    ax.cla()  # Limpar o eixo para atualizar o vetor
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_aspect('equal', 'box')
    
    # Plotando o vetor
    ax.quiver(0, 0, scaled_vec[0], scaled_vec[1], angles='xy', scale_units='xy', scale=1)
    
    # Configurações do gráfico
    ax.set_title(f"Vetor escalado pelo seno de t = {t:.2f}")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    
    plt.grid()
    plt.draw()
    plt.pause(0.1)  # Pausa para permitir a atualização do gráfico