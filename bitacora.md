# Bitacora de trabajo

## Archivos revisados

- README.md
- QUICKSTART.md
- gitignore

## Archivos por revisar

- LICENSE

## Carpetas revisadas

## Carpetas por revisar

- ptu_driver
- ptu_package
- radar_msg
- radar_package
- state_machine

## correciones realizadas

- bitacora asociada: https://github.com/dammr54/SmartFarmRadar/blob/main/bitacora.md

Fecha 19 de octurbe 2026
- se agrego el archivo channel_cal_val: se añadio el archivo de calibración a este repositorio
- se reviso la edicion de los arhcivos README.md y QUICKSTART.md
- se corrigio gitignore
- se fijan los filtros rx_rf_bandwidth y tx_rf_bandwidth en 18 MHz del PlutoSDR al arranque dado que la calibración utiliza un filtro a 20MHz y cambia la configuración
- se corrigieron errores de codigo relacionados a RadarBeamform.action
- se corrigio error relacionado a la dependencia <buildtool_depend>ament_python</buildtool_depend> (eliminado) y a <test_depend>pytest</test_depend> en package.xml reemplazado por "<test_depend>python3-pytest</test_depend>"
- se corrigio el problema C-01 de la bitacora: Quitar el int() de start_offset_samples
- se corrigio el problema C-02 de la bitacora: do_chirp tiene burst_slice como principal, hacerlo como en do_sweep
- se corrigio el problema C-03 de la bitacora: Steer angle en grados, no en radianes
- se corrigio el problema C-04 de la bitacora: Nuevos códigos de ganancia del ADAR1000
- se corrigio el problema C-05 de la bitacora: Tono de TX de 100 kHz exactos con un búfer de 600 muestras (no 6144)
- se corrigio el problema C-06 de la bitacora: Anular por completo la salida TX1
- se corrigio el problema C-07 de la bitacora: Sin bloques viejos en la cola de recepción
- se corrigio el problema C-08 de la bitacora: Tono de TX al fondo de escala del DAC (2**15 - 1 en vez de 2**14)

## Pendientes

- corregir 2 puntos aca: do_chirp tiene burst_slice como principal, hacerlo como en do_sweep
- realizar una nueva calibración
- desktop para rviz2: opcional, estaba en el historial de comandos de uso
- revision del codigo del bunker: `https://github.com/RAL-UC/ral_bunker`: no hay ningun cambio respecto al commit pero si hay 11 commits extras, Commit 628d0b5 “feat: action added”
- state_machine: ral_bunker_msgs: por ahora esta relacionado como repositorio externo
- realizar activación GPS


GPS Mosaic X5 → nmea_tcpclient_driver → /fix (latitud/longitud)
                                          │
                     /imu ─► wgs84_to_utm ◄┘   (ral_gps)
                                 │
                                 ▼
                 /fix_utm  (este/norte en metros, altitud, orientación de la IMU; frame "utm_19H")
                                 │
radar_data → data_processing → radar_cartesian → radar_map_accumulator ◄┘
                                                        │
                                                        ▼
                                          radar_cartesian_accum (puntos + gps_* por punto)
                                                        │
                                                        ▼
                                          mapa_cartesiano (dibuja solo x, y, z)

- Suma pcal a la fase y aplica % 360.0. Además, el paso de fase está definido como phase_step_size = 2.8125, que corresponde a 360/2⁶ porque el desfasador del ADAR1000 es de 6 bits. Si pcal estuviera en radianes, el % 360 no tendría sentido
- como deberia ser el proceso de calibracion con modo "tx"