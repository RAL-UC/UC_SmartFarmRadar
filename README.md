# UC SmartFarm Radar para ROS2 Humble

Este repositorio contiene tres paquetes ROS 2 desarrollados para capturar datos de radar utilizando la plataforma de desarollo de arreglos en fase [**ADALM-PHASER CN0566** de Analog Devices](https://wiki-analog-com.translate.goog/resources/eval/user-guides/circuits-from-the-lab/cn0566?_x_tr_sl=en&_x_tr_tl=es&_x_tr_hl=es&_x_tr_pto=tc) y controlar un **PTU‑C46**, que permite posicionar dinámicamente el radar en distintas direcciones:

### Paquetes incluidos
- **radar_msg**: Definición del mensaje personalizado `RadarData`.
- **radar_package**: Captura, procesamiento y publicación de datos obtenidos desde el radar por medio de conección ethernet.
- **ptu_package**: Interfaz de comunicación serial con el PTU-C46.
- **ptu_routine**: Produce una rutina para controlar orientación y elevación del PTU‑C46.

---

## Requisitos

### Sistema operativo
- Ubuntu 22.04
- ROS 2 Humble Hawksbill

### Dependencias del sistema

Para trabajar con el hardware de radar PhaserX, es necesario instalar las siguientes bibliotecas [Instrucciones detalladas desde Analog Devices](https://wiki.analog.com/resources/tools-software/linux-software/pyadi-iio): 

Paquetes de Python requeridos:
- `pylibiio`
- `pyadi-iio`
- `pyserial`
- `numpy`

Para su instalación se debe seguir el listado de instrucciones de configuración previa de [Build instructions for libiio](https://github.com/analogdevicesinc/libiio/blob/main/README_BUILD.md) hasta antes de clonar el repositorio:
```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libxml2-dev libzstd-dev bison flex libcdk5-dev cmake
sudo apt-get install libaio-dev libusb-1.0-0-dev
sudo apt-get install libserialport-dev libavahi-client-dev
sudo apt-get install doxygen graphviz
sudo apt-get install python3 python3-pip python3-setuptools
```

Luego descargar libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb:
```bash
sudo apt install ./libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb
```

Por último continuar con:
```bash
pip install pylibiio
pip install pyadi-iio
pip install pyserial
pip install numpy
```

### Construcción del workspace

Desde la raíz del workspace:

```bash
colcon build
source install/setup.bash
```

---

## Ejecución
### PTU-C46
#### Paquete `ptu_package`
Para establecer la conexión serial del dispositivo **PTU‑C46** se utiliza un conversor USB a RS-232 modelo TU-S9. 

Ejecuta el nodo de comunicación:

```bash
ros2 run ptu_package ptu_node --ros-args -p serial_port:=/dev/ttyUSB0
```

Para publicar ángulos, los límites por defecto son:
- Pan (orientación horizontal): de **-158° a +158°**
- Tilt (elevación): de **-46° a +31°**

Se permite enviar cualquier comando al **PTU-C46**. Para más detalles, consulte el manual del dispositivo [manual del dispositivo](https://www.sustainable-robotics.com/reference/PTU/PTU-manual-D46-2.15.pdf)
```bash
ros2 topic pub --once /ptu_cmd std_msgs/msg/String "{data: 'pp-1000'}"
```
#### Paquete `ptu_routine`
Ejecuta una rutina predefinida que en base a una señal habilitadora permite realizar el recorrido:

```bash
ros2 run ptu_routine ptu_routine_node
ros2 topic pub --once /allow_routine_ptu std_msgs/msg/Bool "{data: true}"
```

### Paquete `radar_package`
Publica datos de radar desde un archivo .npy
```bash
ros2 run radar_package radar_node
```

Suscribirse para visualizar los datos:
```bash
ros2 run radar_package process_data_node
```

Publicación de datos al topico solo una vez:
```bash
ros2 topic pub --once /allow_sweep std_msgs/msg/Bool "data: true"
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
ros2 bag play UC_SmartFarmRadar/datos/radar_rosbag/intento1.7/intento1.7_0.db3
```
en bucle
```bash
ros2 bag play --loop UC_SmartFarmRadar/datos/radar_rosbag/intento1.7/intento1.7_0.db3
ros2 bag play --loop UC_SmartFarmRadar/datos/data_sync_radar/data_sync_radar_0.db3
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



