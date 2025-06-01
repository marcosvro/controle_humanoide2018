---
applyTo: '**'
---
Modelagem e Representação de Conhecimento para Algoritmos Bioinspirados
Marcos V. R. de Oliveira¹ e Telma W. de Lima Soares²

Resumo. Neste trabalho foi analisado e implementado algumas propostas de algoritmos para controlar um bípede em um ambiente de simulação com física, ambas necessitam de um conjunto de dados para a alimentação do controlador de caminhada, chamaremos este conjunto de estado. Como forma de solução, os controladores emitem outro conjunto de dados utilizado para setar os atuadores do robô, permitindo ou não que ele altere seu estado atual.

Palavras-chave: Inteligência artificial, redes neurais, A3C, humanoide, controlador

INTRODUÇÃO
O uso de robôs baseados na fisionomia humana tem se tornado cada vez mais comum, entregando na maioria dos casos uma eficiência de produção superior à fornecida por trabalho humano. Em alguns casos, pode ser necessário o uso de robôs para a realização da tarefa especificada, como entrar em regiões de alto risco à saúde humana. Por este e outros muitos motivos, robôs cada vez mais robustos vêm sido desenvolvidos, com objetivos específicos e bem definidos. A principal vantagem de modelos humanoides sobre outros tipos terrestres, é a alta mobilidade em terrenos irregulares. No entanto, outras questões de caráter mais simples precisam ser resolvidas, como alcançar um nível de estabilidade na caminhada superior ou pelo menos parecido com o padrão humano.
Alguns trabalhos usam conceitos de física dinâmica para definirem o quão equilibrado o robô se encontra, em um determinado estado[2]. A técnica mais abordada é o Zero Moment Point (ZMP), que diz que um robô está em equilíbrio quando a força resultante de todas suas partes ou ponto de pressão se encontra sobre uma base, formada por pontos em contato com o chão. Neste tipo de estratégia um controlador é implementado para gerar uma função de trajetória para o centro de massa ou uma cinemática de movimento para os atuadores, na qual a soma das forças geradas pela gravidade e momento angular sobre as partes do corpo são anuladas ao manter o ZMP dentro da base de contato. Dado a posição dos pés de apoio em um determinado período de tempo (time step), o objetivo do controlador é garantir que o ZMP estará o máximo de tempo possível sobre as bases em contato com o chão. Como forma de ajuste de equilíbrio, o controlador poderia também alterar as posições futuras dos pés no chão, além de modificar a cinemática ou função de trajetória do centro de massa (CoM)[2].
Outra abordagem que obteve bons resultados em ambiente simulado, é o controlador composto por redes neurais artificiais (RNA)[5][7]. Este método visa “ensinar” uma RNA a manipular os atuadores de um bípede humanoide, i. e. é feito a abstração da lógica de equilíbrio dos estados informados à rede, seguido da inferência de uma ação que possivelmente possa fornecer ao robô uma boa recompensa, com base na função objetivo estabelecida. Para isso, são simulados múltiplos casos com a RNA no controle das ações tomadas pelo robô, onde serão coletadas informações usadas para treinar o modelo. Os pares estado-ação que levarem o modelo a terem melhores resultados são recompensados, mapeando o conjunto universo dos estados em ações moldadas por uma função de recompensa.


2.  Metodologia
Seguindo o escopo abordado no capítulo anterior, uma metodologia de controlador para um bípede humanoide foi implementada. Utilizando conceitos de física e heurísticas para definir parâmetros de forma determinística, com o objetivo de gerar um padrão de caminhada ajustável e com uma alta taxa de soluções produzidas. Para uma melhor organização e modularidade do sistema, o controlador do bípede humanoide pode ser dividido em duas camadas distintas: um controlador de alto nível e um controlador de baixo nível. Esta separação permite uma gestão mais eficaz das diversas tarefas e abstrações envolvidas no controle do robô.

[TODO: Adicionar imagem ilustrando os dois controladores, a imagem deve conter um componente recebendo como entrada o objetivo (ponto no espaço para o qual o robô deve ir) e devolvendo um estado (andando, virando para esquerda ou direita, marchando, parado, etc..) que é a entrada do controlador de baixo nível, que deve produzir os ângulos que os atuadores do robô devem receber]

O controlador de alto nível atua como o "cérebro" do sistema, tomando decisões estratégicas sobre o comportamento geral do robô. Sua responsabilidade é determinar se o robô deve andar para frente, virar à esquerda ou à direita, marchar, ou permanecer parado. Em resumo, o controlador de alto nível lida com a lógica de mais alto nível do comportamento do robô, coordenando suas ações em um nível mais abstrato.

Já o controlador de baixo nível é responsável pela execução precisa das ações definidas pelo controlador de alto nível. Suas responsabilidades incluem:

Cinemática do bípede: Calcular a cinemática inversa do robô, ou seja, determinar os ângulos exatos que as juntas do robô devem ter para alcançar uma determinada posição ou orientação.
Estabilização e equilíbrio: Implementar algoritmos de controle para manter o equilíbrio do robô durante a caminhada e outras ações, compensando perturbações externas e garantindo a estabilidade do robô.

O controlador de baixo nível atua como a interface direta entre o "cérebro" do bípede (controlador de alto nível) e o "corpo" do bípede (atuadores e sensores), garantindo que os comandos sejam executados gerando o movimento esperado.

2.1  Controlador de baixo nível
O controlador baixo pode ser resumido como uma equação matemática em função do tempo, que possui parâmetros que combinados produzem a ação esperada pelo controlador de alto nível. Para melhor descrevermos essas equações vamos remover estes parâmetros, assumindo que o bípede se encontra no estado “andando”.
Assumimos também que o bípede é composto por dois atuadores (ou conjunto de links), um representando a perna de apoio, considerando o eixo do pé como base e o eixo do quadril como extremidade, e outro representando a perna de balanço, tendo o eixo do quadril como base e o eixo do pé como extremidade. O cálculo de cinemática inversa, é o cálculo dos ângulos que devem ser aplicados nos atuadores para mover suas extremidades para um determinado ponto no espaço. O controlador de baixo nível foi divido em dois outros sub-componentes:
Gerador de trajetória do CoM: Tem como objetivo gerar os pontos das extremidades dos atuadores, representados por pH e pF, sendo estas as posições do quadril da perna de apoio e a posição do tornozelo da perna de balanço respectivamente.
Gerador de cinemática: Dados os pontos pH e pF, o gerador de cinemática é responsável por calcular os ângulos que serão aplicados nos atuadores.

2.1.1  Gerador de cinemática
Para facilitar o entendimento do padrão de caminhada, dividimos um ciclo deste padrão em duas partes, cada parte define qual perna estará em contato com o chão (perna de apoio) e qual perna estará se movendo para o próximo ponto (perna de balanço). 


Figura 1. Representação dos links da perna vistos de lado (a) e de frente (b).

Para o calcular os ângulos dos atuadores é assumido que o bípede estará com o tronco sempre ereto, e no caso do eixo pitch, os dois maiores links da perna (link entre as juntas do quadril e joelho, e entre as juntas do joelho e o tornozelo) juntamente com um link imaginário entre as juntas do quadril e do tornozelo formam  um triângulo, onde os lados são conhecidos ou possivelmente calculados (no caso do link imaginário), a Figura 1.a ilustra o cenário apresentado. Uma vez que os lados deste triângulo é conhecido, podemos inferir os ângulos necessários para que o bípede seja posicionado conforme o ponto pH ou pF fornecido. Considerando que o ponto informado seja pH, as equações para {2, 3, 4} são definidas a seguir.

2=tan-1(x/z)+cos-1((a²-b²-c²)/(-2bc))                               (1)

3=cos-1((b²-a²-c²)/(-2ac))-                                      (2)

4=cos-1((c²-a²-b²)/(-2ab))-tan-1(x/z)                               (3)


Em relação ao eixo roll, na Figura 1.b podemos observar um triângulo retângulo , onde os lados adjacentes à 1 são conhecidos, e portanto a equação para a junta roll do tornozelo e quadril é dada por:

1 =tan-1(y/z)                                                       (4)

5= -1                                                          (5)


2.1.2  Gerador de trajetória do CoM
Neste capítulo será descrito como usamos heurísticas e informações do modelo físico do bípede para gerar pH e pF. Dado que conhecemos o padrão de caminhada humano como ilustrado na Figura 2, podemos compor funções que se assimilam ao movimento realizado pelos pontos do pé e quadril. Assumimos que estas funções têm duração definida pela metade do ciclo de caminhada, sendo assim, são necessárias duas delas para completar o período da marcha. Na primeira metade, a perna de apoio recebe o ponto FH(t) enquanto a perna de balanço recebe o ponto FF(t) que serão usados pelo solucionador cinemático definido no Capítulo 2.1. 


Figura 2. Divisão do ciclo de caminhada segundo Perry e Burnfield[1].


2.1.2.1  Função de tragerória baseada no ZMP
Como descrito em [2], podemos assumir que equilíbrio do bípede pode ser alcançado quando: 
A soma das forças ḞA atuando sobre o ponto A (junta do tornozelo), obtida pelo movimento conjunto das partes do corpo, e ḞG, provocada pela ação da gravidade no centro de massa G, são anuladas pela força de contato ḞP aplicada sobre o ponto de pressão P. Assumimos que o pé em contato com o chão não deslizará, portanto, as componentes horizontais [FxA, FyA] serão anuladas pela força de fricção (componentes horizontais de ḞP).
A soma dos momentos do corpo ṀA, e momento gerado por ḞA são anulados por Mzp e pelo posicionamento horizontal de P (caso P esteja sobre a superfície de contato), de forma a compensar o braço de alavanca induzido por Fzp aplicado em [px, py, 0], este ponto é denominado ZMP.

PḞP+GḞG+ṀA+[0,0,Mzp]T+AḞA=0                           (6)

Para satisfazer a condição 2 listada acima, o ZMP calculado precisa residir sobre a superfície de contato com o chão (SC), porém, podem haver situações em que ele precise extrapolar os limites das bordas de SC. Este comportamento resulta em tombamento, e portanto FF(t) e FH(t) devem produzir soluções que nos levam a {ṀA, ḞA, ḞG, G} que não gere P fora da SC. Além disso, a resolução da equação (6) para obter a posição do CoM (Centro de Massa) a partir de um dado ZMP ou vice-versa, pode ser aproximada por modelos como o 3D-LIPM (3D Linear Inverted Pendulum) introduzido em (KAJITA et al., 2001), a partir da seguinte equação apresentada em [2].

аCoM-hgäCoM=аZMP                                                 (7)

Onde а ∊ {x, y}, h representa a altura do CoM (zCoM) e g a aceleração da gravidade (9,80665 m/s²). Dado аCoM atual, poderíamos calcular posições futuras do centro de massa, fazendo com que a aceleração final produzida (ä'CoM) leve a posição final do ZMP (а'ZMP) para o intervalo definido pela SC. Para isso, precisaríamos para cada аCoM, i intermediário, gerar o conjunto de pontos {pH, i e pF, i} não distante do seu sucessor e antecessor (para que a interpolação seja possível dada a velocidade máxima das juntas). Além disso, ao invés de modificar ä'CoM para obter equilíbrio, também poderíamos gerar posições futuras para os pés, fazendo com que а'ZMP coincida com a SC neste momento futuro. Para solucionar este problema podemos utilizar um algoritmo de otimização, onde é necessário a elaboração de uma função a ser minimizada (ou maximizada), gerando no processo um plano de caminhada satisfatório, abordagem utilizada em [2]. Na ausência dessa função e devido à complexidade dos cálculos realizados, a abordagem evolutiva que propunha encontrar estes pontos através de uma busca estocástica, foi desconsiderada por não atender ao requisito do controlador ser executado em tempo real.

2.1.2.2  Função de trajetória proposta
A partir de (7) podemos observar que quando a aceleração do CoM tende para 0, аZMP tende para аCoM. Isso significa que quando o bípede está parado ou se movimentando de maneira com que haja pouca aceleração do CoM, para se manter em equilíbrio, a posição do CoM (x, y) precisa residir sobre a superfície de contato. Como podemos ver no ciclo na (Figura 2), o ciclo de caminhada possui fases onde apenas um dos pés está em contato com o chão, nesse caso, o CoM precisa se mover na direção desses pontos de contato com o chão, e vamos definir FH(t) com esse objetivo. Além disso, precisamos posicionar o pé de balanço na posição futura de contato com o chão e para isso definimos FF(t). Começando por FH(t).

	Tpr(t)=10 (t -Tpasso/2)/Tpasso

FH, x(t)=pHx=(dfeet/2)exp(Tpr(t))-exp(-Tpr(t))exp(Tpr(t))+exp(-Tpr(t))                               (8)

FH, y(t)=pHy= -lhip sin (t/Tpasso)                                     (9)

FH, z(t)=pHz=hhip                                                 (10)

Onde Tpasso é o tempo de um passo e metade do ciclo de caminhada, dfeet é a distância entre os pés na fase double support (Figura 2), lhip é o deslocamento lateral máximo do quadril e hhip é a altura fixa do quadril. Além disso, pHx é dado pela equação tanh (Tangente Hiperbólica), com o objetivo de permitir o deslocamento no eixo x apenas quando próximo da fase double support (Figura 2). A componente pHy representa o deslocamento lateral do quadril com amplitude lhip, e consequentemente o de yCoM.  A função FF(t) é similar a função FH(t) em relação aos componentes horizontais, com a diferença de que pFy é igual a  -pHy para evitar que as pernas se colidam durante a caminhada. Note que pFx também segue o comportamento de uma função tanh, devido ao problema de tentar mover o pé de balanço sem que o ZMP esteja sobre o próximo pé de apoio. A equação para o eixo z de pF é representada pela função sino (11):

FF, z(t)=pFz=ufootexp(-(t-Tpasso/2)²)                                  (11)

Onde ufoot é a altura máxima que o pé de balanço alcançará durante o passo e  um parâmetro que é ajustado manualmente para que o bípede não tente tirar o pé do chão nos momentos iniciais do passo.

2.1.3  Compensação por gravidade
A atuação da aceleração da gravidade sobre as múltiplas partes do corpo provocam torques indesejados sobre as juntas do bípede, além disso, a presença de falha mecânica também pode inviabilizar que a ação gerada resulte no estado esperado. Obtendo o torque gerado pela gravidade, podemos inferir um ângulo de correção proporcional nas juntas mais afetadas por estes fenômenos. Conforme em [2], para cada junta j onde queremos aplicar o ângulo de correção, devemos primeiro calcular m'j e r'CoM, j, a massa total e CoM das partes apoiada por ela respectivamente.
m'j=i∊B'jmi                                                       (12)

r'CoM, j=(i∊B'jrCoM, imi)/m'j                                           (13)

Onde B'j é o conjuntos das partes do corpo apoiadas por j.Com estas informações estamos aptos a calcular o torque exercido pela ação da gravidade em relação ao ponto da junta j.

'j=r'CoM, j0,0,m'jgT                                          (14)

Agora precisamos obter de 'j a componente na direção do eixo de j,

j, g=-'jej                                                     (15)

onde ej representa o vetor unitário que aponta em direção ao eixo de j. Desconsiderando os termos integral e derivativo do controlador, o ângulo de correção a ser aplicado em j, é dado por:

j, g= j, g/Kp                                                    (16)

Onde Kp é a constante proporcional do controlador interno do servo motor. Além disso, na presença de falha mecânica esta constante pode ser ajustada a fim de compensar este tipo de erro. Aplicamos este ângulo de correção apenas nas juntas do joelho e quadril(eixo X, roll) devido ao maior esforço exigido.

2.1  Controlador de alto nível
O controlador de alto nível é responsável por determinar as ações necessárias para fazer com que o bípede cumpra seu objetivo. Ele é uma máquina de estados que a cada frame computa os estados futuros do bípede com base no estado atual e objetivo geral. O objetivo geral é chegar em um determinado ponto no espaço e para isso serão calculados dtarget e target, sendo eles  a distância e o ângulo mínimo necessário para rotacionar o bípede na direção do alvo. Os estados da máquina são “Parado”, “Marchando”, “Caminhando” e “Virando”, e podem ser descritos pela composição das variáveis dfeet, lhip e ufoot utilizadas na função de trajetória do CoM, a seguir vemos cada estado detalhadamente:
Parado: Este é o estado mais simples e é atingido quando todas as variáveis dfeet, lhip e ufoot tem atribuídas o valor 0.
Marchando: A marcha é o estado onde o bípede está deslocando lateralmente o centro de massa entre as superfícies de contato com o chão e o pé de balanço está se movimentando em relação ao eixo z (para cima), no entanto, imóvel em relação ao eixo x (para frente). Este estado pode ser considerado um estado intermediário entre os demais, e pode ser alcançado quando dfeet for igual a 0, lhip e ufoot estiverem com seu valor máximo.
Caminhando: Este é o estado onde o bípede vai estar caminhando para frente e é atingido quando dfeet, lhip e ufoot estiverem em seu valor máximo.
Virando: Este estado é similar ao “Marchando” com a diferença que VAR_ROT vai possuir um valor diferente de 0, sendo -1 para virar a esquerda e 1 para a direita.

3.  Experimentos e resultados
Neste capítulo mostraremos como foram configurados os testes e quais foram os resultados obtidos. Em relação ao ambiente de simulação foi utilizado a Engine V-Rep [6] com o motor físico Bullets v2.8, onde os parâmetros de simulação como o intervalo entre os frames dt, foram personalizados para cada teste. Devido a linguagem padrão da engine ser Lua, tivemos que usar uma interface com o agente implementado em Python, para isso, o Robot Operating System (ROS) nos oferece um meio de comunicação baseado em publishers e subscribers, onde nós podem publicar mensagens em um determinado tópico, para que outros nós possam subscrever funções de callback. 

3.1  Simulações
Nas simulações do controlador heurístico foram usados dt=50 ms(padrão do V-Rep), o que nos oferece uma escala de tempo aproximada a escala de tempo real. A Tabela 1 fornece os parâmetros ajustados manualmente para obter um melhor resultado em relação aos critérios de avaliação definidos a seguir. Estes parâmetros podem requerer modificações na ausência do compensador por gravidade.







Parâmetro
Valor
Tpasso
0,5
dfeet
1,5
lhip
1,4
hhip
17
ufoot
2
Kp
9

Tabela 1. Parâmetros utilizados nas simulações do controle heurístico.

As métricas utilizadas para avaliar os controladores implementados são representadas pelas seguintes variáveis aleatórias:
Duração: Tempo de duração em segundos do episódio até o bípede cair.
Desvio padrão angular do torso: A medida que fornece o desvio médio (em graus) da orientação média do torso em relação aos eixos X (Roll) e Y (Pitch) de uma dada simulação.
Orientação média do torso em Pitch e Roll: Esta variável nos dá a informação de qual foi a orientação média (em graus) do tronco obtida a respeito dos eixos X e Y.
Distância: Distância percorrida em metros pelo bípede durante o episódio.
Foram realizadas 40 simulações utilizando o controle heurístico, onde todas elas obtiveram o valor máximo de tempo de simulação, ou seja, não sofreram queda durante o período de 30 segundos.


Figura 4. Frequência dos intervalos definidos sobre a distância percorrida, em metros.


Figura 5. Frequência dos intervalos definidos sobre os valores de desvio obtidos,
medido em graus.


Figura 6. Frequência dos intervalos definidos sobre a orientação 
(graus X, Y) média do tronco, obtidos de cada simulação.


4.  Discussões e conclusão
Os resultados apresentados pelo controlador heurístico foram bastante promissores, em relação aos critérios de avaliação definido no Capítulo 3.1, resultando em um modelo com baixa variação angular do torso (Figura 5) e com a média angular aceitavelmente próxima à esperada (Figura 6). Além disso, somando ao fato de que nenhuma das 40 simulações sofreram queda, um questionamento foi levantado sobre a robustez desta abordagem de representação para o padrão de caminhada. Com o objetivo de responder tais questões, testes com perturbações externas não relatados neste trabalho foram realizados, obtendo resultados que indicavam a alta sensibilidade à interferências externas, o que é esperado devido o não uso de informações reais (somente estimadas) da simulação para a síntese da ação.


REFERÊNCIAS
[1]  Perry, J. and Burnfield, J.M. “GAIT ANALYSIS. Normal and Pathological Function” (2010). SLACK Inc., Thorofare, 152-153.
[2] M. R. O. de A. Maximo. “Automatic Walking Step Duration through Model Predictive Control”. (2017) Instituto Tecnológico de Aeronáutica, São José dos Campos.
Duration through Model Predictive Control. 2017. 221f. Thesis of Doctor of
Science in Electronic Engineering and Computer Science – Instituto Tecnológico de
Aeronáutica, São José dos Campos.
[3] Y. Wu, E. Mansimov, S. Liao, R. Grosse, J. Ba. “Scalable trust-region method for deep reinforcement learning using Kronecker-factored approximation”. (2017) arXiv:1708.05144
[4] V. Mnih, A. P. Badia, M. Mirza, A. Graves, T. P. Lillicrap, T. Harley, D. Silver, K. Kavukcuoglu. “Asynchronous Methods for Deep Reinforcement Learning”. (2016) arXiv:1602.01783.
[5] X. Bin P. ,  G. Berseth,  K. Yin,  M. van de Panne. “DeepLoco: Dynamic Locomotion Skills Using Hierarchical Deep Reinforcement Learning”. (2017) ACM Transactions on Graphics (Proc. SIGGRAPH 2017).
[6] E. Rohmer, S. P. N. Signgh, M. Freese, “V-REP: a Versatile and Scalable Robot Simulation Framework,” (2013) IEEE/RSJ Int. Conf. on Intelligent Robots and Systems.
[7]  J. Schulman, F. Wolski, P. Dhariwal, A. Radford, O. Klimov “Proximal Policy Optimization Algorithms”. (2017) arXiv:1707.06347.
 


