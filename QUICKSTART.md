# ⚡ QUICKSTART — Comandos de uso frecuente

Apuntes de trabajo diario para el proyecto. La instalación, dependencias y descripción de los paquetes están en el [README](./README.md).

## Índice

1. [Entorno de trabajo](#1-entorno-de-trabajo)
2. [Compilación](#2-compilación)
3. [Ejecución del sistema](#3-ejecución-del-sistema)
4. [Pan-tilt PTU-C46](#4-pan-tilt-ptu-c46)
5. [Experimentos con la plataforma móvil (PC octa)](#5-experimentos-con-la-plataforma-móvil-pc-octa)
6. [Simulación (Gazebo)](#6-simulación-gazebo)
7. [Utilidades del sistema](#7-utilidades-del-sistema)
8. [Git](#8-git)

---

## 1. Entorno de trabajo

### VS Code

Abrir Visual Studio Code en el directorio actual:

```bash
code .
```

### tmux

tmux permite mantener sesiones persistentes en la terminal: las sesiones siguen ejecutándose aunque te desconectes, y puedes volver a ellas para retomar el trabajo en el mismo punto.

**Configuración:** habilitar el mouse para cambiar de pestaña

1. Abrir el archivo de configuración:

   ```bash
   nano ~/.tmux.conf
   ```

2. Añadir la línea:

   ```
   set -g mouse on
   ```

3. Recargar la configuración:

   ```bash
   tmux source-file ~/.tmux.conf
   ```

**Sesiones:**

```bash
# Crear una nueva sesión
tmux new -s magister

# Listar sesiones existentes
tmux ls

# Volver a una sesión existente
tmux attach -t magister
```

- **Desconectar sin cerrar (detach):** `Ctrl + b`, soltar y presionar `d`. La sesión queda en segundo plano.
- **Copiar texto con el mouse activado:** `Shift + selección` para seleccionar como en la terminal normal y luego `Ctrl + Shift + C` para copiar.

---

## 2. Compilación

```bash
# Compilar un solo paquete
colcon build --packages-select ptu_driver

# Cargar el workspace en la terminal actual
source install/local_setup.bash
```

---

## 3. Ejecución del sistema

### Calibración del phaser (uso puntual)

No ejecutar junto con `launch.py`: ambos usan el mismo hardware. Ubicar la HB100 encendida frente al arreglo, en el eje mecánico y a ~1.5 m.

```bash
# Con launch (carga radar.yaml y calibracion.yaml; espera 10 s antes de comenzar)
ros2 launch radar_package calibracion.launch.py

# Con ros2 run (pide presionar Enter antes de comenzar; usa los valores por defecto del nodo)
ros2 run radar_package calibracion

# Repetir solo algunos pasos (los demás se toman de la calibración anterior)
ros2 run radar_package calibracion --ros-args -p "cal_steps:=[gain,phase]"

# Instalar los nuevos .pkl para que radar_node los use
colcon build --packages-select radar_package
```

Resultados en `radar_package/resource/` y gráficos de control en `resource/calibracion_graficos/`. Si el nodo aborta por señal débil, revisar que la HB100 esté encendida y apuntando al arreglo.

### Máquina de estados

```bash
ros2 run state_machine state_machine
ros2 run state_machine active_radar
```

### Tópicos

```bash
# Ver los mensajes de un tópico
ros2 topic echo /nombre_del_topico
```

### Servicios útiles

```bash
# Limpiar el mapa acumulado
ros2 service call /clear_map std_srvs/srv/Empty "{}"
```

### Parámetros

```bash
ros2 param list /radar_node
ros2 param get /radar_node num_chirps
```

---

## 4. Pan-tilt PTU-C46

```bash
# Identificar el puerto serial del conversor USB a RS-232
ls /dev/ttyUSB*

# Ver los ejecutables del driver
ros2 pkg executables ptu_driver
```

> En caso de errores del conversor USB a RS-232 el PTU se ha controlado directamente por terminal serial con `putty`.

---

## 5. Experimentos con la plataforma móvil (PC octa)

### Conexión al PC del robot

1. Conectarse al wifi **asus ral** con cable ethernet desconectado.
2. Conectarse por SSH (clave `2312`):

   ```bash
   ssh octa@10.42.0.1
   ```

> **Tip:** si la conexión se corta con `client_loop: send disconnect: Broken pipe`, los procesos lanzados fuera de tmux se pierden. Trabajar dentro de una sesión de [tmux](#tmux) para que sigan corriendo.

### Secuencia de lanzamiento (Montaje completo)

Cada comando en una terminal (o ventana de tmux) distinta:

```bash
ros2 launch ral_bunker_navigation square_movement.launch.py
ros2 launch radar_package launch.py
ros2 run state_machine state_machine
# Grabar todos los tópicos (Ctrl + C para detener)
ros2 bag record --all -o experimento_radar_ultimo
```

### Mover la plataforma móvil

```bash
ros2 topic pub -1 /allow std_msgs/msg/Bool "data: true"
```

### Copiar los datos al PC propio

Desde **mi propio PC**:

```bash
sudo scp -r octa@10.42.0.1:/home/octa/magister/magister_ws/experimento_radar_ultimo4 /home/dammr/Downloads/
```

Como `scp` se ejecutó con `sudo`, los archivos quedan a nombre de root. Cambiar el dueño:

```bash
sudo chown -R dammr <carpeta>/
```

---

## 6. Simulación (Gazebo)

```bash
sudo apt install -y gazebo \
  ros-humble-gazebo-ros ros-humble-gazebo-ros-pkgs \
  ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher ros-humble-xacro
```

Ruta del paquete de descripción del robot:

```bash
ROBOT_PKG=$(ros2 pkg prefix robot_description)
```

---

## 7. Utilidades del sistema

### Espacio en disco

```bash
df -H
```

### Drivers USB (`linux-modules-extra`)

Ubuntu instala por defecto solo los drivers básicos del kernel; muchos otros vienen en el paquete `linux-modules-extra`. Entre ellos están los necesarios para:

- **Conversores USB a serial** (`usbserial`, `pl2303`, `ftdi_sio`, `cp210x`, `ch341`): crean `/dev/ttyUSB0`. El conversor TU-S9 del PTU usa el chip PL2303, por lo que sin este paquete el PTU no aparece.
- **Adaptadores de red por USB** (`rndis_host`, `cdc_ether`, `cdc_ncm`): por ejemplo, el PlutoSDR conectado por USB (`192.168.2.1`).

```bash
sudo apt install linux-modules-extra-$(uname -r)
```

`$(uname -r)` se reemplaza por la versión del kernel en uso (ej. `6.8.0-138-generic`); el paquete debe corresponder a esa versión.

> **Síntoma de que falta:** `lsusb` muestra el conversor, pero `ls /dev/ttyUSB*` no muestra nada. Suele pasar después de una actualización del kernel. Se soluciona volviendo a ejecutar el comando y reconectando el conversor.

---

## 8. Git

### Descartar cambios locales y dejar todo igual al repositorio remoto

Útil cuando la copia local (por ejemplo, en el PC octa) quedó con cambios o conflictos y `git pull` falla: deja la rama exactamente igual a la de GitHub.

> ⚠️ **Irreversible.** Se pierden:
> - los cambios sin commitear;
> - los commits locales que no se subieron con `git push`;
> - los archivos y carpetas no versionados. Solo se conservan los que están en el `.gitignore` (`build/`, `install/`, `log/`, `.vscode/`, `datos/`).

> ⚠️ **Rosbags dentro del repositorio:** grábalos solo en `datos/` (como indica el README). Esa carpeta está en el `.gitignore`, así que `git clean -fd` no la toca; un rosbag guardado en cualquier otra carpeta del repositorio sí se borra. No usar `git clean -fdx`: la `-x` borra también lo que está en el `.gitignore`, incluida `datos/`.

1. Revisar qué se va a borrar (no borra nada, solo lista):

   ```bash
   git clean -fdn
   ```

2. Identificar la rama que se quiere dejar igual a GitHub. Normalmente es la rama en la que estás trabajando:

   ```bash
   git branch --show-current
   ```

3. Resetear, reemplazando `<rama>` por ese nombre (ej. `main` o `nueva_version`):

   ```bash
   git reset --hard                # descarta cambios en archivos versionados
   git clean -fd                   # elimina archivos y carpetas no versionados
   git checkout <rama>             # cambia a la rama (los dos pasos anteriores evitan que falle)
   git fetch origin                # descarga lo último de GitHub, sin aplicarlo
   git reset --hard origin/<rama>  # deja la rama idéntica a su versión en GitHub
   ```

> **¿Qué rama elegir?** Un repositorio guarda varias versiones del código en paralelo, llamadas *ramas*. En este proyecto, `main` es la versión principal y `nueva_version` es donde se desarrollan los cambios nuevos. El reset deja tu copia igual a **una sola** de ellas: la que escribas en lugar de `<rama>`.
>
> Por ejemplo, si en el PC octa necesitas lo último de `nueva_version` pero escribes `main`, el PC quedará con el código de `main` y sin los cambios de `nueva_version`.
>
> Solo hay que cambiar `<rama>` en esos dos comandos; los otros tres se copian tal cual.

Ejemplo para la rama `nueva_version`:

```bash
git checkout nueva_version
git reset --hard origin/nueva_version
```
