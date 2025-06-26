import signal
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tensorboardX import SummaryWriter
from controlador import Controlador
import threading
import vrep
import os
import sys
import select
import termios
import tty
import subprocess

class SimulacaoManager:
    def __init__(self, max_episodes=40):
        self.max_episodes = max_episodes
        self.stat_writer = SummaryWriter(comment="amostras-controle-euristico")
        self.reset()

        # Inicia a conexão com o V-REP usando subprocess para controlar o processo
        porta = 19700
        coppelia_cmd = (
            f"/home/marcos/Documents/CoppeliaSim_Edu_V4_7_0_rev2_Ubuntu22_04/coppeliaSim.sh "
            f"-gREMOTEAPISERVERSERVICE_{porta}_FALSE_FALSE "
            f"/home/marcos/Documents/dev/controle_humanoide2018/teste_09_03.ttt"
        )
        self.coppelia_proc = subprocess.Popen(
            coppelia_cmd,
            shell=True,
        )
        time.sleep(20)
        self.clientID=vrep.simxStart('127.0.0.1',porta,True,True,5000,5) # Connect to V-REP

        if self.clientID!=-1:
            # start the simulation:
            vrep.simxStartSimulation(self.clientID,vrep.simx_opmode_blocking)
        else:
            print ("Não foi possivel estabelecer conexão com o vrep")
            self._finaliza_processo()
            exit()

    def reset(self):
        self.cont_samples = 1
        self.duracoes = []
        self.dp_ori_x = []
        self.dp_ori_y = []
        self.me_ori_x = []
        self.me_ori_y = []
        self.distancias = []
        self.historico_distancias_por_tempo = []
        self.historico_ori_x_por_tempo = []
        self.historico_ori_y_por_tempo = []

    def run(self):
        control = Controlador(simulador_enable=True, gravity_compensation_enable=True)
        # Executa control.run() em uma thread separada
        control_thread = threading.Thread(target=control.run)
        control_thread.start()

        for episode in range(1, self.max_episodes + 1):
            print(f"=== Episódio {episode} ===")
            control.activate = False
            control.state = 'IDDLE'
            time.sleep(3)
            vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_blocking)
            time.sleep(3)
            print("Iniciando simulação no V-REP")

            tempo_inicio = time.time()
            amostras_ori_x = []
            amostras_ori_y = []
            distancias_durante_run = []
            dist = 0
            control.activate = True
            skip_run = False

            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                while control_thread.is_alive():
                    if self._key_pressed():
                        ch = sys.stdin.read(1)
                        if ch == 's':
                            print("Run pulada pelo usuário.")
                            skip_run = True
                            break

                    time.sleep(control.simTransRate)

                    amostras_ori_x.append(control.robo_roll)
                    amostras_ori_y.append(control.robo_pitch)
                    dist = control.distancia_ponto_alvo
                    distancias_durante_run.append(control.distancia_ponto_alvo)

                    # Critério de queda
                    if (control.state == "FALLEN"):
                        break

                    # Timeout de segurança
                    if control.chegou_no_alvo and time.time() - tempo_inicio > 5:
                        break
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

            if skip_run:
                vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
                continue

            # Reinicia simulação no VREP
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

            duracao = time.time() - tempo_inicio
            print(f"salvando dados da simulação {episode}")
            self.duracoes.append(duracao)
            variacoes_abs_ori_x = np.abs(amostras_ori_x)
            variacoes_abs_ori_y = np.abs(amostras_ori_y)
            media_variacoes_abs_ori_x = np.mean(variacoes_abs_ori_x)
            media_variacoes_abs_ori_y = np.mean(variacoes_abs_ori_y)
            desvio_padrao_variacoes_abs_ori_x = np.std(variacoes_abs_ori_x)
            desvio_padrao_variacoes_abs_ori_y = np.std(variacoes_abs_ori_y)
            self.dp_ori_x.append(desvio_padrao_variacoes_abs_ori_x)
            self.dp_ori_y.append(desvio_padrao_variacoes_abs_ori_y)
            self.me_ori_x.append(media_variacoes_abs_ori_x)
            self.me_ori_y.append(media_variacoes_abs_ori_y)
            self.distancias.append(dist)
            self.historico_ori_x_por_tempo.append(amostras_ori_x)
            self.historico_ori_y_por_tempo.append(amostras_ori_y)
            self.historico_distancias_por_tempo.append(distancias_durante_run)

            self.stat_writer.add_scalar("Tempo de simulação/Episode", duracao, episode)
            self.stat_writer.add_scalar("Desvio padrão da variação angular absoluta em X/Episode", desvio_padrao_variacoes_abs_ori_x, episode)
            self.stat_writer.add_scalar("Desvio padrão da variação angular absoluta em Y/Episode", desvio_padrao_variacoes_abs_ori_y, episode)
            self.stat_writer.add_scalar("Média da variação angular absoluta em X/Episode", media_variacoes_abs_ori_x, episode)
            self.stat_writer.add_scalar("Média da variação angular absoluta em Y/Episode", media_variacoes_abs_ori_y, episode)
            self.stat_writer.add_scalar("Distância do alvo/Episode", dist, episode)


        # Garante que a thread finalize
        if control_thread.is_alive():
            # Se saiu do loop por queda ou timeout, aguarda thread terminar
            control_thread.join(timeout=1)
        self.finaliza()

    def finaliza(self):
        time.sleep(3)
        print("Finalizando teste")
        print(len(self.duracoes))
        print(len(self.dp_ori_x))
        print(len(self.dp_ori_y))
        print(len(self.me_ori_x))
        print(len(self.me_ori_y))
        print(len(self.distancias))

        self._plot_and_save(pd.Series(np.array(self.duracoes)), 'Duração da simulação em segundos', 'Tempo (s)', 'runs/time_duration.png')
        self._plot_and_save(pd.Series(np.array(self.dp_ori_x)), 'Desvio padrão da variação angular absoluta (X)', 'Desvio padrão da variação angular absoluta (X)', 'runs/dp_x.png')
        self._plot_and_save(pd.Series(np.array(self.dp_ori_y)), 'Desvio padrão da variação angular absoluta (Y)', 'Desvio padrão da variação angular absoluta (Y)', 'runs/dp_y.png')
        self._plot_and_save(pd.Series(np.array(self.me_ori_x)), 'Média da variação angular absoluta (X)', 'Média da variação angular absoluta (X)', 'runs/mean_x.png')
        self._plot_and_save(pd.Series(np.array(self.me_ori_y)), 'Média da variação angular absoluta (Y)', 'Média da variação angular absoluta (Y)', 'runs/mean_y.png')
        self._plot_and_save(pd.Series(np.array(self.distancias)), 'Distância do alvo', 'Distância', 'runs/dist.png')

        self._plot_value_vs_tempo(self.historico_distancias_por_tempo,
                                  'Distância do alvo ao longo do tempo por run',
                                  'Distância do alvo',
                                  'runs/distancia_vs_tempo.png')
        self._plot_value_vs_tempo(self.historico_ori_x_por_tempo,
                                  'Oriêntação do eixo Roll ao longo do tempo por run',
                                  'Oriêntação do eixo Roll',
                                  'runs/orientacao_x_vs_tempo.png')
        self._plot_value_vs_tempo(self.historico_ori_y_por_tempo,
                                  'Oriêntação do eixo Pitch ao longo do tempo por run',
                                  'Oriêntação do eixo Pitch',
                                  'runs/orientacao_y_vs_tempo.png')

        self.stat_writer.close()
        self._finaliza_processo()

    def _finaliza_processo(self):
        """Finaliza o processo atual."""
        self.coppelia_proc.kill()
        exit(0)

    def _key_pressed(self):
        """
        Retorna True se alguma tecla foi pressionada no terminal, sem bloquear a execução.
        """
        dr, dw, de = select.select([sys.stdin], [], [], 0)
        return dr != []
    
    def _plot_value_vs_tempo(self, amostras, titulo, yLabel, filename):
        """
        Plota um gráfico onde cada linha representa a evolução de um determinado valor ao longo do tempo em cada run.
        """
        plt.figure()
        for idx, valores_da_run in enumerate(amostras):
            plt.plot(valores_da_run, linewidth=0.25)
        plt.title(titulo)
        plt.xlabel('Tempo (frame)')
        plt.ylabel(yLabel)
        # plt.legend()
        plt.grid(True)
        plt.savefig(filename)
        plt.clf()

    def _plot_and_save(self, series, title, ylabel, filename):
        if len(series) == 0:
            return
        series.plot.hist(grid=True, bins=10, rwidth=0.9, color='#607c8e')
        plt.title(title)
        plt.xlabel('Frequência')
        plt.ylabel(ylabel)
        plt.grid(axis='y', alpha=0.75)
        plt.savefig(filename)
        plt.clf()

if __name__ == "__main__":
    sim = SimulacaoManager(max_episodes=42)
    sim.run()