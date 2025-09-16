### Abrir Visual Studio Code en directorio actual:
```bash
code .
```
### COnfiguración de tmux
Es posible editar archivos de configuración predeterminada. Dentro de las configuraciones se incluye añadir la funcionalidad del mouse para cambiar de pestaña, crear sesiones para mantener sesiones persistentes en la terminal (las sesiones continúan ejecutándose incluso si te desconectas), opciones de cierre de sesione y apertura para volver a trabajar en un punto determinado
```bash
nano ~/.tmux.conf
set -g mouse on
tmux source-file ~/.tmux.conf
```
crear nueva sesion:
```bash
tmux new -s magister
```

desconectar sin cerrar: Ctrl + b, luego suelta y presiona d
Esto "detaches" la sesión, dejándola en segundo plano.

Volver a una sesion existente
```bash
tmux ls
tmux attach -t magister
```


atajo de teclado: shift + selección, selecciona como el default y luego Ctrl + Shift + C para copiar

ros2 bag play --loop UC_SmartFarmRadar/datos/data_sync_radar/data_sync_radar_0.db3


colcon build --packages-select ptu_package


```bash
ros2 run ptu_controller control_node \
  --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baudrate:=9600
```

creacion de launch
ros2 launch radar_package launch.py


ls /dev/ttyUSB*

ros2 topic pub --once /ptu_cmd std_msgs/msg/String "{data: 'pp-1000'}"
ros2 pkg executables ptu_package


```bash
ros2 topic pub /trigger_sweep std_msgs/msg/Bool "data: true"
```

Conexion PC octa
conectar a wifi asus ral, desconectar ethernet
ssh octa@10.42.0.1
clave 2312
ros2 launch ral_bunker_navigation square_movement.launch.py
ros2 launch radar_package launch.py
ros2 run state_machine state_machine
ros2 bag record --all -o experimento_radar_ultimo


Para mover el bunker:

ros2 topic pub -1 /allow std_msgs/msg/Bool data:\ true\

Grabar topicos:

ros2 bag record --all -o <nombre_archivo>
Ctrl + C para cerrar
ros2 bag play <nombre_del_bag>
ros2 topic echo /nombre_del_topico

Desde MI propio PC:
sudo scp -r octa@10.42.0.1:/home/octa/magister/magister_ws/experimento_radar_ultimo4 /home/dammr/Downloads/

Cambiar owner:
sudo chown -R dammr oli/

consulta memoria
df -H

client_loop: send disconnect: Broken pipe


ros2 launch radar_package launch.py

ros2 topic pub --once /start_scan std_msgs/msg/Bool "{data: true}"


sudo apt install gazebo ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros
sudo apt install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro

sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-robot-state-publisher

source install/local_setup.bash

ROBOT_PKG=$(ros2 pkg prefix robot_description)


modulos de red usb
sudo apt install linux-modules-extra-$(uname -r)


para eliminar cambios en local y hacer pull para que quede igual que el repositorio remoto
Borra cambios en archivos versionados
Elimina archivos/directorios no versionados
Forzar actualización desde remoto
```bash
git reset --hard
git clean -fd
git checkout main
git fetch origin
git reset --hard origin/main
```