# Webots_YOLOv8

Pacote ROS 2 de visao para o Webots. Ele combina a deteccao YOLOv8 com um
pipeline de segmentacao por cores para fronteira do campo, obstaculos e pontos
de linha.

## Estrutura corrigida

- `Webots_YOLOv8/pipeline/`: algoritmo novo de segmentacao (padrao).
- `Webots_YOLOv8/yolo_simulation.py`: no ROS 2 e integracao com YOLO/IPM.
- `Webots_YOLOv8/segmentacao.py`: algoritmo anterior, opcional para comparacao.
- `modelo/best.pt`: pesos usados pelo YOLO.
- `recursos/*.csv`: calibracao HSV do campo e do branco.

## Compilar

No diretorio do workspace, carregue sua distribuicao ROS 2 e remova os
artefatos gerados no caminho antigo antes de recompilar:

```bash
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

O aviso sobre letras maiusculas no nome `Webots_YOLOv8` e apenas uma convencao
do ROS 2; o nome foi preservado para manter compatibilidade com os comandos e
launch files existentes.

## Executar

```bash
ros2 launch Webots_YOLOv8 vision.launch.py
```

Para testar somente o novo pipeline, sem carregar o YOLO:

```bash
ros2 launch Webots_YOLOv8 vision.launch.py enable_yolo:=false
```

O executavel espera os pacotes ROS 2 listados em `package.xml` e o pacote
Python `ultralytics`. Caso necessario:

```bash
python3 -m pip install --user ultralytics
```

## Teste rapido

Depois de compilar e carregar `install/setup.bash`:

```bash
python3 -c "import Webots_YOLOv8; print(Webots_YOLOv8.__file__)"
ros2 pkg executables Webots_YOLOv8
```

O segundo comando deve mostrar `Webots_YOLOv8 finder`.
