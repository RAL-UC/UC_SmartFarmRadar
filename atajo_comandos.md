abrir visual studio code en directorio actual:
```bash
code
```

configuracion tmux
```bash
nano ~/.tmux.conf
set -g mouse on
tmux source-file ~/.tmux.conf
```

atajo de teclado: shift + selección, selecciona como el default y luego Ctrl + Shift + C para copiar


colcon build --packages-select ptu_package


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