---title: Roteiro de Apresentação
---description: Roteiro de apresentação do artigo "Modelagem e Representação Matemática Calibrável para Marcha de Bípede Humanoide"

## Slide 1: Capa
- **Título:** Modelagem e Representação Matemática Calibrável para Marcha de Bípede Humanoide
- **Autores:** Marcos V. R. de Oliveira¹, Telma W. de Lima Soares²
- **Instituições**
- **Data**

---

## Slide 2: Roteiro da Apresentação
- Introdução
- Fundamentação teórica
- Metodologia
- Experimentos e resultados
- Discussão e conclusão

---

## Slide 3: Motivação
- Crescente uso de robôs humanoides
- Vantagens em ambientes complexos
- Desafios: estabilidade, adaptação e controle eficiente

---

## Slide 4: Objetivo do Trabalho
- Desenvolver um controlador calibrável para marcha de bípede humanoide
- Gerar padrões de caminhada estáveis e eficientes
- Modularidade e facilidade de calibração

---

## Slide 5: Resumo do Artigo
- Metodologia baseada em modelagem física e heurísticas ajustáveis
- Controlador em duas camadas: alto e baixo nível
- Testes em ambiente simulado (V-Rep)
- Resultados: estabilidade, baixa variação angular, robustez

---

## Slide 6: Palavras-chave
- bípede, robô, humanoide, controlador, caminhada, marcha, zero-moment-point, heurístico, off-line

---

## Slide 7: Introdução
- Robôs humanoides: aplicações e desafios
- Equilíbrio e estabilidade na marcha
- Abordagens tradicionais: ZMP, redes neurais, controle dinâmico

---

## Slide 8: Fundamentação Teórica – ZMP
- Conceito de Zero Moment Point (ZMP)
- Importância para equilíbrio dinâmico
- Ilustração do ZMP em marcha

---

## Slide 9: Outras Abordagens
- Controle por redes neurais artificiais (RNA)
- Aprendizado por reforço
- Limitações e vantagens

---

## Slide 10: Metodologia – Visão Geral
- Controlador dividido em alto e baixo nível
- Modularidade e separação de responsabilidades

---

## Slide 11: Arquitetura do Controlador
- Diagrama: interação entre alto e baixo nível
- Responsabilidades de cada camada

---

## Slide 12: Controlador de Alto Nível
- Máquina de estados: Parado, Marchando, Caminhando, Virando
- Decisão estratégica baseada em objetivo e orientação

---

## Slide 13: Controlador de Baixo Nível
- Execução precisa dos comandos
- Cinemática inversa e estabilização
- Interface com atuadores e sensores

---

## Slide 14: Gerador de pose
- Geração dos pontos pH (quadril) e pF (tornozelo)
- Funções heurísticas para trajetória

---

## Slide 15: Equações Matemáticas
- Equações para cálculo dos ângulos dos atuadores (pitch, roll)
- Equação do ZMP e sua aplicação

---

## Slide 16: Funções de Trajetória
- Função tanh para deslocamento no eixo x
- Função seno para deslocamento lateral
- Função sino para altura do pé de balanço

---

## Slide 17: Parâmetros do Controlador
- Tabela 1: Parâmetros de ajuste mínimo
- Tabela 2: Parâmetros ajustáveis para calibração

---

## Slide 18: Processo de Calibração
- Ajuste de Tpasso, lhip, ufoot, dfeet, Maxyall
- Estratégia incremental para estabilidade

---

## Slide 19: Ambiente de Simulação
- V-Rep com motor físico Bullet v2.8
- Comunicação via Python e ROS
- Descrição da configuração experimental

---

## Slide 20: Configuração Inicial da Simulação
- Imagem da cena inicial (Figura 5)
- Posição do objetivo e tarefa do robô

---

## Slide 21: Métricas de Avaliação
- Duração do episódio
- Média e desvio padrão da variação angular (pitch e roll)
- Distância percorrida

---

## Slide 22: Resultados – Distância ao Objetivo
- Gráfico de frequência das distâncias finais (Figura 6a)
- Gráfico da variação da distância ao longo do tempo (Figura 6b)

---

## Slide 23: Resultados – Variação Angular Média
- Gráficos de variação angular média em X (roll) e Y (pitch) (Figura 7)

---

## Slide 24: Resultados – Desvio Padrão Angular
- Gráficos de desvio padrão da variação angular em X e Y (Figura 8)

---

## Slide 25: Análise dos Resultados
- Nenhuma simulação apresentou queda
- Objetivo sempre atingido
- Baixa variação angular e boa estabilidade

---

## Slide 26: Robustez e Adaptação
- Controlador robusto frente a perturbações simuladas
- Facilidade de ajuste dos parâmetros

---

## Slide 27: Limitações e Desafios
- Sensibilidade a perturbações externas reais
- Possibilidade de melhorias com sensores em tempo real e controle adaptativo

---

## Slide 28: Discussão e Conclusão
- Controlador heurístico gera marcha estável e eficiente
- Resultados positivos em ambiente simulado
- Base sólida para avanços futuros

---

## Slide 29: Trabalhos Futuros
- Testes em hardware real
- Incorporação de sensores e controle adaptativo
- Exploração de aprendizado de máquina para ajuste automático de parâmetros

---

## Slide 30: Referências
- [1] Perry, J. and Burnfield, J.M. “GAIT ANALYSIS. Normal and Pathological Function” (2010).
- [2] M. R. O. de A. Maximo. “Automatic Walking Step Duration through Model Predictive Control”. (2017).
- [3] Y. Wu et al. “Scalable trust-region method for deep reinforcement learning...” (2017).
- [4] X. Bin P. et al. “DeepLoco: Dynamic Locomotion Skills Using Hierarchical Deep RL” (2017).
- [5] E. Rohmer et al. “V-REP: a Versatile and Scalable Robot Simulation Framework,” (2013).
- [6] J. Schulman et al. “Proximal Policy Optimization Algorithms”. (2017).

---

## Slide 31: Agradecimentos e Perguntas
- Agradecimentos aos colaboradores e instituições
- Espaço para perguntas

---