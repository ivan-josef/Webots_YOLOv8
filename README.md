    # Pipeline de Visão Computacional e Controle de Robôs com Webots + ROS 2 + YOLOv8

Este projeto demonstra um pipeline de visão computacional e controle de robôs integrado, utilizando a simulação **Webots** e o framework **ROS 2**.  
Ele permite que um robô virtual se mova e use uma câmera para detectar objetos com o modelo **YOLOv8** em tempo real.

---

## Funcionalidades

- **Simulação Realista**: Utiliza o simulador de robótica **Webots** para criar um ambiente virtual e um robô móvel, o **AUREA**.  
- **Comunicação ROS 2**: Estabelece uma conexão bidirecional entre a simulação Webots e a rede ROS 2, permitindo o fluxo de dados dos sensores e o envio de comandos de controle.  
- **Detecção de Objetos com YOLOv8**: Um nó ROS 2 dedicado processa o fluxo de vídeo da câmera do robô para detectar objetos e desenhar caixas delimitadoras (*bounding boxes*).  
- **Controle de Teleop**: Controla o movimento do robô e o pan da câmera usando comandos de teclado.  
- **Visualização em Tempo Real**: A imagem processada é publicada em um tópico para visualização externa com o `rqt_image_view`.  

---

## Tecnologias e Dependências

- **ROS 2 Humble**: Middleware de comunicação robótica.  
- **Webots**: Ambiente de simulação de robótica.  
- **Python 3.10**: Linguagem principal dos nós.  
- **YOLOv8 & Ultralytics**: Framework de detecção de objetos.  
- **OpenCV & cv_bridge**: Processamento de imagem e integração ROS ↔ OpenCV.  

---

## Estrutura do Projeto

O projeto possui dois pacotes principais ROS 2:

- **`my_package`**  
  - Contém o driver do robô para Webots (`my_robot_driver.py`) e arquivos de configuração.
  - Contém também o nó de controle do robô (`keyboard_teleop.py`)
  - Gerencia a conexão com a simulação.  

- **`object_finder`**  
  - Contém o nó de visão computacional (`connecting_and_showing.py`).  
  - Utiliza o YOLOv8 para detecção de objetos.  
  - Publica as *bounding boxes* e imagens processadas.  

---

## Como Executar

### Pré-requisitos
- **Webots** instalado (versão compatível com ROS 2 Humble).  
- **ROS 2 Humble** instalado e configurado.  
- Um **workspace ROS 2** (ex.: `~/ros2_ws`).  

---

### 1. Clonar o Repositório
Dentro da pasta `src` do seu workspace ROS 2:
```bash
cd ~/ros2_ws/src
git clone <URL_DO_REPOSITORIO>
```

### 2. Adicionar o Modelo YOLOv8
Baixe seu modelo **best.pt** e salve-o na pasta raiz do pacote object_finder.

### 3. Compile o Projeto
```bash
cd ~/ros2_ws
colcon build
```

### 4. Configurar o ambiente
```bash
source install/setup.bash
```

### 5. Executar o pipeline
Abra 3 terminais e execute:
- Terminal 1 – Driver do Robô e Simulação
  ```bash
  ros2 launch my_package my_robot_driver
  ```
- Terminal 2 – Nó de Detecção
  ```bash
  ros2 run object_finder finder
  ```
- Terminal 3 – Controle de Teleop
  ```bash
  ros2 run my_package keyboard_controller
  ```

  ### 6. Visualizar as detecções
  ```bash
  ros2 run rqt_image_view rqt_image_view 
  ```
  E selecione o tópico ***processed_image_topic*** para ver as caixas de detecção em tempo real.

  ## 🤝 Contribuição

  Sinta-se à vontade para contribuir
