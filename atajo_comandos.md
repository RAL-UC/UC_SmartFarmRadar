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


ls /dev/ttyUSB*

ros2 topic pub --once /ptu_cmd std_msgs/msg/String "{data: 'pp-1000'}"
ros2 pkg executables ptu_package
