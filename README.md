# UC SmartFarm Radar para ROS 2 Humble

Este repositorio contiene paquetes de ROS 2 desarrollados para capturar datos de radar en banda X utilizando la plataforma de desarollo de arreglos en fase [**ADALM-PHASER CN0566** de Analog Devices](https://wiki-analog-com.translate.goog/resources/eval/user-guides/circuits-from-the-lab/cn0566?_x_tr_sl=en&_x_tr_tl=es&_x_tr_hl=es&_x_tr_pto=tc) y controlar un posicionador **PTU‑C46** para ajustar dinámicamente el radar hacia distintas direcciones.

## 📦 Paquetes Incluidos

| Paquete | Descripción |
| :--- | :--- |
| **`radar_msg`** | Estructura estandarizada de acciones y mensajes personalizados |
| **`radar_package`** | Captura, procesamiento y publicación de datos de radar vía Ethernet |
| **`ptu_driver`** | Interfaz de comunicación serial RS-232 con el PTU-C46 |
| **`ptu_package`** | Rutinas predefinidas para controlar la orientación y elevación del PTU-C46 |
| **`state_machine`** | Máquina de estado para orquestar las acciones de los distintos dispotivos (Robot movil, radar, PTU) |

---

## 🛠️ Requisitos e Instalación

### Sistema Base
- **S.O.:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill

### Dependencias del Sistema y Hardware

Dependencias de compilación y librerías del PhaserX: [Instrucciones detalladas desde Analog Devices](https://wiki.analog.com/resources/tools-software/linux-software/pyadi-iio).

Debe seguir el listado de instrucciones de configuración previa de [Build instructions for libiio](https://github.com/analogdevicesinc/libiio/blob/main/README_BUILD.md) hasta antes de clonar el repositorio:

```bash
sudo apt-get update
sudo apt-get install build-essential
sudo apt-get install libxml2-dev libzstd-dev bison flex libcdk5-dev cmake
sudo apt-get install libaio-dev libusb-1.0-0-dev
sudo apt-get install libserialport-dev libavahi-client-dev
sudo apt-get install doxygen graphviz
sudo apt-get install python3 python3-pip python3-setuptools
```

Descarga el paquete libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb y ejecutalo:

```bash
sudo apt install ./libiio-0.26.ga0eca0d-Linux-Ubuntu-22.04.deb
```

Por último instala las dependecias de python:

```bash
pip install pylibiio
pip install pyadi-iio
pip install pyserial
pip install numpy
```

### Construcción del workspace

Desde la raíz del workspace (ej. /UC_SmartFarmRadar):

```bash
colcon build
source install/setup.bash
```
---

## 🚀 Guía de Ejecución

### 1. PhaserX

**Captura y Procesamiento (`radar_package`):**
Un único archivo de lanzamiento centraliza la puesta en marcha del radar, la PTU, la visualización de mapas, el procesamiento de los datos y los simuladores de depuración

```bash
ros2 launch radar_package launch.py
```

**Mensajes Personalizados (`radar_msg`):**
Estructura y estandariza el intercambio de información y la ejecución de acciones entre dispositivos. Incluye nodos ejecutables a modo de ejemplo para depuración.

```bash
ros2 interface show radar_msg/msg/RadarData
```

Nodos de ejemplo:

```bash
ros2 run radar_msg publish_radar_data
ros2 run radar_msg subscribe_radar_data
```

### 2. PTU-C46

> **Nota:** Para La conexión serial del dispositivo **PTU‑C46** se utiliza un conversor USB a RS-232 modelo TU-S9. 

**Driver de Comunicación (`ptu_driver`):**

```bash
ros2 run ptu_driver ptu_node_driver --ros-args -p serial_port:=/dev/ttyUSB0
```

*Límites de movimiento:* Pan (horizontal) **-158° a +158°** | Tilt (vertical) **-46° a +31°**.

Se permite enviar cualquier comando al **PTU-C46**. Para más detalles, consulte el [manual del dispositivo](https://www.sustainable-robotics.com/reference/PTU/PTU-manual-D46-2.15.pdf)

```bash
ros2 topic pub --once /ptu_cmd std_msgs/msg/String "{data: 'pp-1000'}"
```

**Rutina Automática (`ptu_package`):**
Ejecuta una rutina predefinida en base a una señal habilitadora:

```bash
ros2 topic pub --once /start_scan std_msgs/msg/Bool "{data: true}"
```

## 📹 Grabación y Reproducción de Rosbags

### Reproducir datos (.db3)

```bash
# Reproducción simple
ros2 bag play UC_SmartFarmRadar/datos/<nombre_carpeta>/<nombre_archivo>.db3

# Reproducción en bucle
ros2 bag play --loop UC_SmartFarmRadar/datos/<nombre_carpeta>/<nombre_archivo>.db3
```

### Grabar datos

```bash
# Grabar todos los tópicos activos en una ubicación específica
ros2 bag record -a -o ~/Desktop/magister_ws/UC_SmartFarmRadar/datos/<nombre_de_la_medicion>
```

## ⚙️ Configuración del Entorno y Red

### Configuración de Red para el Radar

Para garantizar la comunicación Ethernet con el PhaserX:
1. Ve a **Settings → Network → Wired**.
2. Añade o edita un perfil en la pestaña **IPv4**.
3. Selecciona la opción **Shared to other computers**. Esto asignará dinámicamente una IP al puerto Ethernet.


### Carga Automática de Entorno (`~/.bashrc`)
Para evitar ejecutar `source` manualmente en cada terminal, añade las siguientes líneas a tu archivo `~/.bashrc`:

```bash
# cargar entorno base de ROS 2 Humble
#source ~/ros2_humble/install/setup.bash # compilada desde código fuente
source /opt/ros/humble/setup.bash # APT
# cargar workspace del proyecto (con validación de existencia)
if [ -f ~/Desktop/magister_ws/UC_SmartFarmRadar/install/setup.bash ]; then
    source ~/Desktop/magister_ws/UC_SmartFarmRadar/install/setup.bash
fi
```

### Herramientas utiles para desarrollador
- **VS Code** (Entorno de desarrollo)
- **tmux** (Gestión de múltiples terminales)
- **PuTTY** Pruebas directas por terminal serial con el PTU

Para más información leer también [QUICKSTART](./QUICKSTART.md)



