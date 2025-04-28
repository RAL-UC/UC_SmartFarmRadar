# UC SmartFarm Radar para ROS2 (Humble)

Este repositorio contiene tres paquetes ROS 2 desarrollados para capturar datos de radar utilizando la plataforma de desarollo de arreglos en fase **PhaserX** y controlar un **PTU‑D46**, que permite posicionar dinámicamente el radar en distintas direcciones:

### Paquetes incluidos
- **radar_msg**: Definición del mensaje personalizado `RadarData`.
- **radar_package**: Captura, procesamiento y publicación de datos obtenidos desde el radar por medio de conección ethernet.
- **ptu_controller**: Control de orientación y elevación del PTU‑D46 mediante comandos seriales.

---

## Requisitos

### Sistema operativo
- Ubuntu 22.04
- ROS 2 Humble Hawksbill

### Dependencias del sistema

Para trabajar con el hardware de radar (PhaserX), es necesario instalar las siguientes bibliotecas [Instrucciones detalladas desde Analog Devices](https://wiki.analog.com/resources/tools-software/linux-software/pyadi-iio): 

seguir el listado de instrucciones de configuración previa de [Build instructions for libiio](https://github.com/analogdevicesinc/libiio/blob/main/README_BUILD.md) hasta antes de clonar el repositorio. 
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libxml2-dev libzstd-dev bison flex libcdk5-dev cmake
sudo apt-get install libaio-dev libusb-1.0-0-dev
sudo apt-get install libserialport-dev libavahi-client-dev
sudo apt-get install doxygen graphviz
sudo apt-get install python3 python3-pip python3-setuptools
```

Luego descargar libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb.
```bash
sudo apt install ./libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb
```

Por ultimo continuar con 
```bash
pip install pylibiio
pip install pyadi-iio
```

Paquetes de Python requeridos:
- `pylibiio`
- `pyadi-iio`
- `pyserial`
- `numpy`

### Construcción del workspace

Desde la raíz del workspace:

```bash
colcon build
source install/setup.bash
```

---

## Ejecución
### Paquete `ptu_controller`

Ejecuta el nodo de control del PTU‑D46:

```bash
ros2 run ptu_controller control_node \
  --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p baudrate:=9600
```

Para publicar ángulos, los límites por defecto son:
- Pan (orientación horizontal): de **-158° a +158°**
- Tilt (elevación): de **-46° a +31°**

```bash
# Orientar a +30° en pan
ros2 topic pub /pan_angle  std_msgs/Float64 "{data: 30.0}"

# Inclinar a -20° en tilt
ros2 topic pub /tilt_angle std_msgs/Float64 "{data: -20.0}"
```

### Paquete `radar_package`
Publica datos de radar desde un archivo .npy
```bash
ros2 run radar_package radar_node
```

Suscribirse para visualizar los datos:
```bash
ros2 run radar_package process_data
```

publicacion de datos al topico:
```bash
ros2 topic pub /trigger_sweep std_msgs/msg/Bool "data: true"
```

publicar solo una vez
```bash
ros2 topic pub --once /trigger_sweep std_msgs/msg/Bool "data: true"
```

### Paquete `radar_msg`
Este paquete define el mensaje personalizado `RadarData` para estructurar la información del radar. Además, incluye nodos ejecutables a modo de ejemplo. La definición del mensaje puede visualizarse con el siguiente comando:

```bash
ros2 interface show radar_msg/msg/RadarData
```
Nodos de ejemplo:

```bash
ros2 run radar_msg publish_radar_data
ros2 run radar_msg subscribe_radar_data
```

correr rosbag

```bash
ros2 bag play /home/dammr/Downloads/radar/intento1.7/intento1.7_0.db3
```

Para cargar el entorno de ros2 de forma automatica en tu computador y espacio de trabajo añade lo siguiente a tu archivo `~/.bashrc`
```bash
source ~/ros2_humble/install/setup.bash
source ~/Desktop/magister_ws/install/setup.bash
```

### Herramientas utiles para desarrollador
- Visual Studio Code
- tmux

Para establecer comunicación de forma correcta con el radar se debe establecer un perfil en settings -> network -> wired -> + -> IPV4 -> shared to other computers -> add

De esta forma se le estara asignando una ip al puerto

### Pun-tilt
mientras tanto se ha controlador por medio del terminal serial `putty`



