import matplotlib.pyplot as plt
import numpy as np
import math
import time
from robot import Robo
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryPlotter:
    def __init__(self):
        # Criar instância do robô
        self.robo = Robo(tempo_passo=6)
        self.espaco_entre_pelves = 3
        
        # Listas para armazenar as trajetórias
        self.trajectories = {
            'right_hip': [],    # Quadril da perna direita
            'right_foot': [],   # Pé da perna direita
            'left_hip': [],     # Quadril da perna esquerda
            'left_foot': []     # Pé da perna esquerda
        }
        
        # Posições globais acumuladas para continuidade
        self.global_positions = {
            'right_hip': [0, -self.espaco_entre_pelves, self.robo.pos_inicial_pelves[2]],
            'right_foot': [0, -self.espaco_entre_pelves - self.robo.pos_inicial_pelves[1], 0],
            'left_hip': [0, self.espaco_entre_pelves, self.robo.pos_inicial_pelves[2]],
            'left_foot': [0, self.espaco_entre_pelves + self.robo.pos_inicial_pelves[1], 0]
        }
        
        # Contador de passos para rastrear mudanças de perna
        self.step_count = 0
        
    def update_trajectories_realtime(self, dt=0.01, total_time=10.0):
        """
        Simula o movimento do robô e plota os pontos de trajetória em tempo real
        """
        current_time = 0.0

        plt.ion()
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')
        colors = {
            'right_hip': 'red',
            'right_foot': 'darkred',
            'left_hip': 'blue',
            'left_foot': 'darkblue'
        }
        labels = {
            'right_hip': 'Quadril Direito',
            'right_foot': 'Pé Direito',
            'left_hip': 'Quadril Esquerdo',
            'left_foot': 'Pé Esquerdo'
        }

        # Flag para detectar mudança de perna
        last_perna = self.robo.perna

        while current_time < total_time:

            if not self.robo.esta_levantando_o_pe():
                self.robo.marchar()
            elif not self.robo.esta_andando_velocidade_max():
                self.robo.acelera_frente()

            self.robo.computa_angulos("WALKING")
            x = (self.robo.t_state * self.robo.nEstados) / self.robo.tempoPasso
            pelv_point, foot_point = self.robo.getTragectoryPoint(x)
            
            # Detectar mudança de perna
            if last_perna != self.robo.perna:
                # # Atualizar posições globais quando há troca de perna
                # if last_perna == 0:  # Perna esquerda estava no chão
                #     # Pé esquerdo se torna a nova base
                #     self.global_positions["left_foot"] = self.global_positions["left_foor"].copy()
                # else:  # Perna direita estava no chão
                #     # Pé direito se torna a nova base
                #     self.global_positions["right_foot"] = self.global_positions["right_hip"].copy()
                last_perna = self.robo.perna

            # Determinar qual perna está no chão (perna de apoio)
            support_leg = "right" if self.robo.perna == 1 else "left"
            swing_leg = "left" if self.robo.perna == 1 else "right"

            # Calcular posições globais
            if support_leg == "right":
                # Perna direita no chão (perna de apoio)
                # Quadril direito: posição fixa + deslocamento local
                right_hip_global = [
                    self.global_positions["right_hip"][0] + self.robo.deslocamentoXpes/2. + pelv_point[0],
                    -self.espaco_entre_pelves + (pelv_point[1] - self.robo.pos_inicial_pelves[1]),
                    pelv_point[2]
                ]
                # Pé direito: permanece fixo na posição global
                right_foot_global = self.global_positions["right_foot"].copy()
                
                # Perna esquerda balançando
                # Quadril esquerdo: posição fixa + deslocamento local
                left_hip_global = [
                    self.global_positions["left_hip"][0] + self.robo.deslocamentoXpes/2. + pelv_point[0],
                    self.espaco_entre_pelves + (pelv_point[1] - self.robo.pos_inicial_pelves[1]),
                    pelv_point[2]
                ]
                # Pé esquerdo: posição do quadril esquerdo + deslocamento local
                left_foot_global = [
                    self.global_positions["left_foot"][0] + (self.robo.deslocamentoXpes/2. + pelv_point[0]) + (self.robo.deslocamentoXpes/2. + foot_point[0]),
                    self.robo.pos_inicial_pelves[1] + self.espaco_entre_pelves + (pelv_point[1] - self.robo.pos_inicial_pelves[1]) + (foot_point[1] - self.robo.pos_inicial_pelves[1]),
                    self.robo.altura - foot_point[2]
                ]
            else:
                # Perna esquerda no chão (perna de apoio)
                # Quadril esquerdo: posição fixa + deslocamento local
                left_hip_global = [
                    self.global_positions["left_hip"][0] + self.robo.deslocamentoXpes/2. + pelv_point[0],
                    self.espaco_entre_pelves - (pelv_point[1] - self.robo.pos_inicial_pelves[1]),
                    pelv_point[2]
                ]
                # Pé esquerdo: permanece fixo na posição global
                left_foot_global = self.global_positions["left_foot"].copy()

                # Perna direita balançando
                # Quadril direito: posição fixa + deslocamento local
                right_hip_global = [
                    self.global_positions["right_hip"][0] + self.robo.deslocamentoXpes/2. + pelv_point[0],
                    -self.espaco_entre_pelves - (pelv_point[1] - self.robo.pos_inicial_pelves[1]),
                    pelv_point[2]
                ]
                # Pé direito: posição do quadril esquerdo + deslocamento local
                right_foot_global = [
                    self.global_positions["right_foot"][0] + (self.robo.deslocamentoXpes/2. + pelv_point[0]) + (self.robo.deslocamentoXpes/2. + foot_point[0]),
                    -self.robo.pos_inicial_pelves[1] - self.espaco_entre_pelves - (pelv_point[1] - self.robo.pos_inicial_pelves[1]) - (foot_point[1] - self.robo.pos_inicial_pelves[1]),
                    self.robo.altura - foot_point[2]
                ]

            # Atualizar posições globais
            self.global_positions["right_hip"][0] = right_hip_global[0]
            self.global_positions["right_foot"][0] = right_foot_global[0]
            self.global_positions["left_hip"][0] = left_hip_global[0]
            self.global_positions["left_foot"][0] = left_foot_global[0]

            # Armazenar pontos
            self.trajectories['right_hip'].append(right_hip_global)
            self.trajectories['right_foot'].append(right_foot_global)
            self.trajectories['left_hip'].append(left_hip_global)
            self.trajectories['left_foot'].append(left_foot_global)

            # --- Atualização em tempo real ---
            ax.clear()
            for key in self.trajectories:
                points = np.array(self.trajectories[key])
                if len(points) > 0:
                    ax.plot(points[:, 0], points[:, 1], points[:, 2], 
                           color=colors[key], linewidth=2, label=labels[key])
                    ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], 
                              color=colors[key], s=50, marker='o')
            
            ax.set_xlabel('X (cm)')
            ax.set_ylabel('Y (cm)')
            ax.set_zlabel('Z (cm)')  # type: ignore
            ax.set_title(f'Trajetória 3D - Perna de Apoio: {"Direita" if support_leg == "right" else "Esquerda"}')
            ax.legend()
            ax.grid(True)
            plt.pause(dt)

            current_time += dt
        plt.ioff()
        plt.show()
    
    def convert_to_global(self, local_point, target):
        """
        Converte pontos locais para coordenadas globais considerando a continuidade
        """
        x, y, z = local_point
        
        # Para o quadril da perna de apoio, usar posição acumulada como base
        if target in ["right_hip", "left_hip"]:
            base_pos = self.global_positions[target]
            return [base_pos[0] + x, base_pos[1] + y, base_pos[2] + z]
        
        # Para o pé de balanço, usar o quadril da perna de balanço como base
        elif target in ["right_foot", "left_foot"]:
            hip_target = target.replace("foot", "hip")
            base_pos = self.global_positions[hip_target]
            return [base_pos[0] + x, base_pos[1] + y, base_pos[2] + z]
        
        return local_point
    
    def plot_3d_trajectories(self):
        """
        Plota as trajetórias em 3D
        """
        fig = plt.figure(figsize=(12, 8))
        ax = fig.add_subplot(111, projection='3d')  # type: ignore
        
        # Converter listas de pontos para arrays numpy
        trajectories_array = {}
        for key, points in self.trajectories.items():
            if points:
                trajectories_array[key] = np.array(points)
        
        # Plotar cada trajetória com cores diferentes
        colors = {
            'right_hip': 'red',
            'right_foot': 'darkred',
            'left_hip': 'blue',
            'left_foot': 'darkblue'
        }
        
        labels = {
            'right_hip': 'Quadril Direito',
            'right_foot': 'Pé Direito',
            'left_hip': 'Quadril Esquerdo',
            'left_foot': 'Pé Esquerdo'
        }
        
        for key, points in trajectories_array.items():
            if len(points) > 0:
                ax.plot(points[:, 0], points[:, 1], points[:, 2], 
                       color=colors[key], linewidth=2, label=labels[key])
                
                # Marcar pontos inicial e final
                ax.scatter(points[0, 0], points[0, 1], points[0, 2], 
                          color=colors[key], s=50, marker='o')
                ax.scatter(points[-1, 0], points[-1, 1], points[-1, 2], 
                          color=colors[key], s=50, marker='s')
        
        # Configurar o gráfico
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')  # type: ignore
        ax.set_title('Trajetórias 3D dos Pés e Quadris do Robô Humanoide')
        ax.legend()
        
        plt.tight_layout()
        plt.show()

    def plot_2d_projections(self):
        """
        Plota projeções 2D das trajetórias (vista superior e lateral)
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        trajectories_array = {}
        for key, points in self.trajectories.items():
            if points:
                trajectories_array[key] = np.array(points)
        
        colors = {
            'right_hip': 'red',
            'right_foot': 'darkred',
            'left_hip': 'blue',
            'left_foot': 'darkblue'
        }
        
        labels = {
            'right_hip': 'Quadril Direito',
            'right_foot': 'Pé Direito',
            'left_hip': 'Quadril Esquerdo',
            'left_foot': 'Pé Esquerdo'
        }
        
        # Vista superior (X-Y)
        for key, points in trajectories_array.items():
            if len(points) > 0:
                ax1.plot(points[:, 0], points[:, 1], 
                        color=colors[key], linewidth=2, label=labels[key])
                ax1.scatter(points[0, 0], points[0, 1], 
                           color=colors[key], s=30, marker='o')
                ax1.scatter(points[-1, 0], points[-1, 1], 
                           color=colors[key], s=30, marker='s')
        
        ax1.set_xlabel('X (cm)')
        ax1.set_ylabel('Y (cm)')
        ax1.set_title('Vista Superior (X-Y)')
        ax1.legend()
        ax1.grid(True)
        ax1.set_aspect('equal')
        
        # Vista lateral (X-Z)
        for key, points in trajectories_array.items():
            if len(points) > 0:
                ax2.plot(points[:, 0], points[:, 2], 
                        color=colors[key], linewidth=2, label=labels[key])
                ax2.scatter(points[0, 0], points[0, 2], 
                           color=colors[key], s=30, marker='o')
                ax2.scatter(points[-1, 0], points[-1, 2], 
                           color=colors[key], s=30, marker='s')
        
        ax2.set_xlabel('X (cm)')
        ax2.set_ylabel('Z (cm)')
        ax2.set_title('Vista Lateral (X-Z)')
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    """
    Função principal para executar a simulação e plotagem
    """
    print("Iniciando simulação de trajetórias do robô humanoide (tempo real)...")
    
    # Criar plotter de trajetórias
    plotter = TrajectoryPlotter()
    
    # Simular movimento por 5 segundos
    print("Simulando movimento por 5 segundos...")
    plotter.update_trajectories_realtime(dt=0.02, total_time=12.0)
    
    # Plotar resultados
    print("Plotando trajetórias 3D...")
    plotter.plot_3d_trajectories()
    
    print("Plotando projeções 2D...")
    plotter.plot_2d_projections()
    
    print("Simulação concluída!")

if __name__ == "__main__":
    main() 