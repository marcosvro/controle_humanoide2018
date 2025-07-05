#!/usr/bin/env python3
"""
Programa para regenerar gráficos a partir de dados do TensorBoard.
Lê os dados salvos pelo roda_simulacao.py e regenera as mesmas imagens.
"""

import os
import sys
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
import glob

class PlotRegenerator:
    def __init__(self, runs_dir="runs"):
        self.runs_dir = runs_dir
        
    def list_available_runs(self):
        """Lista todas as pastas de execução disponíveis."""
        if not os.path.exists(self.runs_dir):
            print(f"Diretório {self.runs_dir} não encontrado!")
            return []
            
        runs = []
        for item in os.listdir(self.runs_dir):
            item_path = os.path.join(self.runs_dir, item)
            if os.path.isdir(item_path):
                # Verifica se tem arquivo de eventos do TensorBoard
                event_files = glob.glob(os.path.join(item_path, "events.out.tfevents.*"))
                if event_files:
                    runs.append(item)
        
        return sorted(runs)
    
    def load_tensorboard_data(self, run_name):
        """Carrega dados do TensorBoard de uma execução específica."""
        run_path = os.path.join(self.runs_dir, run_name)
        event_files = glob.glob(os.path.join(run_path, "events.out.tfevents.*"))
        
        if not event_files:
            raise FileNotFoundError(f"Nenhum arquivo de eventos encontrado em {run_path}")
        
        # Usa o primeiro arquivo de eventos encontrado
        event_file = event_files[0]
        print(f"Carregando dados de: {event_file}")
        
        # Carrega os dados do TensorBoard
        ea = EventAccumulator(event_file)
        ea.Reload()
        
        # Extrai os dados das métricas
        data = {}
        
        # Lista de métricas que sabemos que existem
        metrics = [
            "Tempo_de_simulação/Episode",
            "Desvio_padrão_da_variação_angular_absoluta_em_X/Episode",
            "Desvio_padrão_da_variação_angular_absoluta_em_Y/Episode", 
            "Média_da_variação_angular_absoluta_em_X/Episode",
            "Média_da_variação_angular_absoluta_em_Y/Episode",
            "Distância_do_alvo/Episode"
        ]
        
        for metric in metrics:
            if metric in ea.Tags()['scalars']:
                events = ea.Scalars(metric)
                values = [event.value for event in events]
                steps = [event.step for event in events]
                data[metric] = {'values': values, 'steps': steps}
                print(f"  {metric}: {len(values)} pontos")
            else:
                print(f"  Aviso: Métrica '{metric}' não encontrada")
        
        return data
    
    def regenerate_plots(self, run_name, output_dir=None):
        """Regenera todos os gráficos para uma execução específica."""
        if output_dir is None:
            output_dir = os.path.join(self.runs_dir, run_name)
        
        # Cria o diretório de saída se não existir
        os.makedirs(output_dir, exist_ok=True)
        
        # Carrega os dados
        data = self.load_tensorboard_data(run_name)
        
        if not data:
            print("Nenhum dado encontrado para gerar gráficos!")
            return
        
        print(f"Gerando gráficos em: {output_dir}")
        
        # Mapeia os nomes das métricas para os nomes dos arquivos
        metric_to_file = {
            "Tempo_de_simulação/Episode": "time_duration.png",
            "Desvio_padrão_da_variação_angular_absoluta_em_X/Episode": "dp_x.png",
            "Desvio_padrão_da_variação_angular_absoluta_em_Y/Episode": "dp_y.png",
            "Média_da_variação_angular_absoluta_em_X/Episode": "mean_x.png",
            "Média_da_variação_angular_absoluta_em_Y/Episode": "mean_y.png",
            "Distância_do_alvo/Episode": "dist.png"
        }
        
        # Mapeia os nomes das métricas para os títulos dos gráficos
        metric_to_title = {
            "Tempo_de_simulação/Episode": "Duração da simulação em segundos",
            "Desvio_padrão_da_variação_angular_absoluta_em_X/Episode": "Desvio padrão da variação angular absoluta (X)",
            "Desvio_padrão_da_variação_angular_absoluta_em_Y/Episode": "Desvio padrão da variação angular absoluta (Y)",
            "Média_da_variação_angular_absoluta_em_X/Episode": "Média da variação angular absoluta (X)",
            "Média_da_variação_angular_absoluta_em_Y/Episode": "Média da variação angular absoluta (Y)",
            "Distância_do_alvo/Episode": "Distância do alvo"
        }
        
        # Mapeia os nomes das métricas para os labels do eixo Y
        metric_to_ylabel = {
            "Tempo_de_simulação/Episode": "Tempo (s)",
            "Desvio_padrão_da_variação_angular_absoluta_em_X/Episode": "Desvio padrão da variação angular absoluta (X)",
            "Desvio_padrão_da_variação_angular_absoluta_em_Y/Episode": "Desvio padrão da variação angular absoluta (Y)",
            "Média_da_variação_angular_absoluta_em_X/Episode": "Média da variação angular absoluta (X)",
            "Média_da_variação_angular_absoluta_em_Y/Episode": "Média da variação angular absoluta (Y)",
            "Distância_do_alvo/Episode": "Distância"
        }
        
        # Gera os gráficos de histograma
        for metric, filename in metric_to_file.items():
            if metric in data:
                self._plot_histogram(
                    data[metric]['values'],
                    metric_to_title[metric],
                    metric_to_ylabel[metric],
                    os.path.join(output_dir, filename)
                )
                print(f"  Gerado: {filename}")
        
        # Gera os gráficos de evolução temporal (se tivermos dados suficientes)
        # Nota: Os dados do TensorBoard não incluem o histórico temporal completo,
        # apenas os valores finais de cada episódio. Para gerar os gráficos de evolução
        # temporal, precisaríamos dos dados brutos de cada simulação.
        
        print("Gráficos regenerados com sucesso!")
        print("Nota: Os gráficos de evolução temporal (distancia_vs_tempo.png, orientacao_x_vs_tempo.png, orientacao_y_vs_tempo.png)")
        print("não podem ser regenerados apenas com os dados do TensorBoard, pois requerem os dados brutos de cada simulação.")
    
    def _plot_histogram(self, values, title, ylabel, filename):
        """Plota um histograma similar ao original."""
        if len(values) == 0:
            print(f"  Aviso: Nenhum dado para {title}")
            return
            
        plt.figure(figsize=(10, 6))
        # Usa matplotlib diretamente para ter controle sobre os eixos
        plt.hist(values, bins=10, rwidth=0.9, color='#607c8e', edgecolor='black')
        plt.title(title)
        plt.xlabel(ylabel)  # O valor da métrica vai no eixo X
        plt.ylabel('Frequência')  # Frequência vai no eixo Y
        plt.grid(axis='y', alpha=0.75)
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close()
    
    def regenerate_all_runs(self, output_base_dir="regenerated_plots"):
        """Regenera gráficos para todas as execuções disponíveis."""
        runs = self.list_available_runs()
        
        if not runs:
            print("Nenhuma execução encontrada!")
            return
        
        print(f"Encontradas {len(runs)} execuções:")
        for i, run in enumerate(runs, 1):
            print(f"  {i}. {run}")
        
        for run in runs:
            print(f"\nProcessando: {run}")
            output_dir = os.path.join(output_base_dir, run)
            try:
                self.regenerate_plots(run, output_dir)
            except Exception as e:
                print(f"  Erro ao processar {run}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Regenera gráficos a partir de dados do TensorBoard")
    parser.add_argument("--runs-dir", default="runs", help="Diretório contendo as pastas de execução")
    parser.add_argument("--run-name", help="Nome específico da execução para processar")
    parser.add_argument("--output-dir", help="Diretório de saída para os gráficos")
    parser.add_argument("--list-runs", action="store_true", help="Lista todas as execuções disponíveis")
    parser.add_argument("--all-runs", action="store_true", help="Regenera gráficos para todas as execuções")
    
    args = parser.parse_args()
    
    regenerator = PlotRegenerator(args.runs_dir)
    
    if args.list_runs:
        runs = regenerator.list_available_runs()
        if runs:
            print("Execuções disponíveis:")
            for i, run in enumerate(runs, 1):
                print(f"  {i}. {run}")
        else:
            print("Nenhuma execução encontrada!")
        return
    
    if args.all_runs:
        regenerator.regenerate_all_runs()
        return
    
    if args.run_name:
        regenerator.regenerate_plots(args.run_name, args.output_dir)
    else:
        # Modo interativo
        runs = regenerator.list_available_runs()
        if not runs:
            print("Nenhuma execução encontrada!")
            return
        
        print("Execuções disponíveis:")
        for i, run in enumerate(runs, 1):
            print(f"  {i}. {run}")
        
        try:
            choice = int(input("\nEscolha uma execução (número): ")) - 1
            if 0 <= choice < len(runs):
                selected_run = runs[choice]
                print(f"\nProcessando: {selected_run}")
                regenerator.regenerate_plots(selected_run, args.output_dir)
            else:
                print("Escolha inválida!")
        except (ValueError, KeyboardInterrupt):
            print("\nOperação cancelada.")

if __name__ == "__main__":
    main() 