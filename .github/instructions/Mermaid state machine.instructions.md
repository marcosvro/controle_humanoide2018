---
config:
  theme: default
  themeVariables:
    background: '#ffffff'
    primaryColor: '#e3e3e3'
    edgeLabelBackground: '#ffffff'
    fontFamily: ''
    noteBkgColor: '#f9fbe7'
    noteBorderColor: '#bdb76b'
---
stateDiagram
  direction LR
  [*] --> Parado
  Parado --> Marchando:objetivo definido
  Marchando --> Caminhando:não chegou no objetivo
  Marchando --> Virando:necessário ajuste de orientação
  Marchando --> Parado:objetivo atingido
  Caminhando --> Marchando:objetivo atingido
  Caminhando --> Virando:necessário ajuste de orientação
  Virando --> Marchando:ajuste de orientação concluído
  note left of Parado
    dfeet = 0d
    lhip = 0
    ufoot = 0
    RT = 0
  end note
  note left of Marchando 
    dfeet = 0
    lhip = máx
    ufoot = máx
    RT = 0
  end note
  note left of Caminhando 
    dfeet = máx
    lhip = máx
    ufoot = máx
    RT = 0
  end note
  note left of Virando 
    dfeet = 0
    lhip = máx
    ufoot = máx
    RT ≠ 0
  end note
