# Regenerate Plots - Regenerador de Gráficos

Este programa lê os dados salvos pelo `roda_simulacao.py` no TensorBoard e regenera as mesmas imagens de gráficos que são geradas ao final da execução da simulação.

## Funcionalidades

O programa pode regenerar os seguintes gráficos:
- `time_duration.png` - Duração da simulação em segundos
- `dp_x.png` - Desvio padrão da variação angular absoluta (X)
- `dp_y.png` - Desvio padrão da variação angular absoluta (Y)
- `mean_x.png` - Média da variação angular absoluta (X)
- `mean_y.png` - Média da variação angular absoluta (Y)
- `dist.png` - Distância do alvo

**Nota**: Os gráficos de evolução temporal (`distancia_vs_tempo.png`, `orientacao_x_vs_tempo.png`, `orientacao_y_vs_tempo.png`) não podem ser regenerados apenas com os dados do TensorBoard, pois requerem os dados brutos de cada simulação.

## Instalação

Certifique-se de ter as dependências instaladas:

```bash
pip install -r requirements.txt
```

## Uso

### 1. Listar execuções disponíveis

```bash
python regenerate_plots.py --list-runs
```

### 2. Regenerar gráficos para uma execução específica

```bash
python regenerate_plots.py --run-name "Jun07_23-06-44_marcos-Nitro-AN515-52amostras-controle-euristico"
```

### 3. Regenerar gráficos para uma execução específica com diretório de saída personalizado

```bash
python regenerate_plots.py --run-name "Jun07_23-06-44_marcos-Nitro-AN515-52amostras-controle-euristico" --output-dir "meus_graficos"
```

### 4. Regenerar gráficos para todas as execuções

```bash
python regenerate_plots.py --all-runs
```

### 5. Modo interativo

```bash
python regenerate_plots.py
```

O programa irá listar todas as execuções disponíveis e permitir que você escolha uma para processar.

## Estrutura de Diretórios

O programa espera encontrar as execuções no diretório `runs/` com a seguinte estrutura:

```
runs/
├── Jun07_23-06-44_marcos-Nitro-AN515-52amostras-controle-euristico/
│   ├── events.out.tfevents.1749348404.marcos-Nitro-AN515-52
│   ├── time_duration.png
│   ├── dp_x.png
│   ├── dp_y.png
│   ├── mean_x.png
│   ├── mean_y.png
│   └── dist.png
└── outras_execucoes/
    └── ...
```

## Parâmetros

- `--runs-dir`: Diretório contendo as pastas de execução (padrão: "runs")
- `--run-name`: Nome específico da execução para processar
- `--output-dir`: Diretório de saída para os gráficos
- `--list-runs`: Lista todas as execuções disponíveis
- `--all-runs`: Regenera gráficos para todas as execuções

## Exemplo de Saída

```
Carregando dados de: runs/Jun07_23-06-44_marcos-Nitro-AN515-52amostras-controle-euristico/events.out.tfevents.1749348404.marcos-Nitro-AN515-52
  Tempo de simulação/Episode: 42 pontos
  Desvio padrão da variação angular absoluta em X/Episode: 42 pontos
  Desvio padrão da variação angular absoluta em Y/Episode: 42 pontos
  Média da variação angular absoluta em X/Episode: 42 pontos
  Média da variação angular absoluta em Y/Episode: 42 pontos
  Distância do alvo/Episode: 42 pontos
Gerando gráficos em: runs/Jun07_23-06-44_marcos-Nitro-AN515-52amostras-controle-euristico
  Gerado: time_duration.png
  Gerado: dp_x.png
  Gerado: dp_y.png
  Gerado: mean_x.png
  Gerado: mean_y.png
  Gerado: dist.png
Gráficos regenerados com sucesso!
```

## Dependências

- `tensorboard`: Para ler os dados do TensorBoard
- `numpy`: Para manipulação de arrays
- `pandas`: Para manipulação de dados
- `matplotlib`: Para geração de gráficos
- `tensorboardX`: Para compatibilidade com os dados salvos 