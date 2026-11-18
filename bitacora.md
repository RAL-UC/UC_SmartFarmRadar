# Bitácora de trabajo

Registro del repositorio ROS 2 (`RAL-UC/UC_SmartFarmRadar`, rama `nueva_version`). Se lleva junto con la bitácora del documento y los scripts, [`dammr54/SmartFarmRadar` · bitacora.md](https://github.com/dammr54/SmartFarmRadar/blob/main/bitacora.md), de la que se toman los códigos de cada cambio. Último cruce entre ambas: 18-11-2026.

## Cómo usar esta bitácora

- **Códigos:** son los de la bitácora asociada, así que un mismo cambio lleva el mismo código en ambas. C = código (scripts y nodos), CF = configuración (YAML y ROS 2), D = documento (texto de `cadena-senal-cn0566`).
- **Pendientes sin código:** los que solo tocan este repositorio (Bunker, GPS, RViz2) van en «Propios del repositorio» y no llevan código.
- **Marcar un avance:** cambia `[ ]` por `[x]`, agrega la fecha en formato día-mes-año, por ejemplo `[x] 19-10-2026`, pasa la entrada a «Hechos» y repite el cambio en la bitácora asociada.

---

## Revisión del repositorio

- **Archivos revisados:** `README.md`, `QUICKSTART.md`, `.gitignore`, `LICENSE`.
- **Archivos por revisar:** 
- **Archivos nuevos por revisar** (nodo de calibración): `calibracion.yaml`, `calibracion.launch.py`, `calibracion_node.py` y `phaser_functions.py`.
- **Carpetas revisadas:** ninguna todavía.
- **Carpetas por revisar:** `ptu_driver`, `ptu_package`, `radar_msg`, `radar_package` y `state_machine`.

---

## Pendientes

### Código

- [ ] **C-02 (seguimiento) · Corregir 2 puntos: `do_chirp` todavía no queda igual que `do_sweep`**
  - Anotación original: «corregir 2 puntos acá: do_chirp tiene burst_slice como principal, hacerlo como en do_sweep».
  - Al cruzar con el código (18-11-2026) se ven dos diferencias en `radar_node.py`:
    - `do_sweep` inicia `time_data` con `np.ones(...) * 1e-10`, pero `do_chirp` con `np.ones(...)`: en `do_chirp`, el relleno (*zero padding*) vale 1 en vez de ≈ 0.
    - En ambas funciones, el comentario dice «desde la columna 0: los visualizadores aplican la ventana en `mat[:, :good_ramp_samples]`», pero el chirp se copia desde la columna `start_offset_samples`. Hay que corregir el comentario o el índice.

- [ ] **C-10 · Ver el espectro en frecuencia de batido, centrado en el tono**
  - Detalle y opciones (desplazar la señal antes de la FFT o cambiar solo el eje) en la bitácora asociada.

- [ ] **C-11 · Polar plot**
  - Falta precisar qué cambiar o revisar en el gráfico polar (PPI) de `radar_polar_visualizer`.

### Configuración (YAML y ROS 2)

- [ ] **CF-01 (verificación) · Nueva calibración y prueba en hardware con la IF de 1.4 GHz**
  - Recalibrar con `cal_rx_lo_hz: 1.4e9`: los `.pkl` de `radar_package/resource/` se hicieron con 2.2 GHz.
  - Medir la fuga del LO en 11.4–11.9 GHz, comparar el C/N0 del GPS con el radar encendido y apagado, y recalibrar `range_offset_hz` con un blanco a distancia conocida.
  - Detalle del cambio en «Hechos», sección «Observaciones o medio hecho».

- [ ] **CF-04 · Distancia entre antenas: `element_spacing_m` de 14 mm o 15 mm**
  - Confirmar el espaciado físico de la placa. Hoy vale 0.014 en `radar.yaml` y como valor por defecto en `radar_node.py`, `calibracion_node.py` y los dos visualizadores (la calibración lo toma de `radar.yaml`).

- [ ] **CF-05 · Ancho de haz de TX en el comentario de `radar.yaml`**
  - Los 24° del comentario no corresponden a la Vivaldi WA5VJB (≈ 8–10 dBi), que tiene un haz más ancho.

- [ ] **CF-06 · Método CFAR por defecto: `average`**
  - `processing.yaml` define `cfar_method: "average"`, pero `radar_polar_visualizer` no lo lee: parte de su selector (`RadioButtons`, `active=0`). Leerlo desde el YAML o eliminarlo de ahí.

- [ ] **CF-07 · Archivos de configuración y `phaser_examples.py`**
  - Falta precisar qué revisar.

### Propios del repositorio

- [ ] **Desktop para RViz2** (opcional): estaba en el historial de comandos de uso.
- [ ] **Código del Bunker** ([`RAL-UC/ral_bunker`](https://github.com/RAL-UC/ral_bunker)): no hay cambios respecto del commit usado, pero hay 11 commits más; entre ellos, `628d0b5` «feat: action added».
- [ ] **`state_machine`:** por ahora `ral_bunker_msgs` se relaciona como repositorio externo.
- [ ] **Activación del GPS.** Flujo previsto:

  ```text
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
  ```

---

## Hechos

Pendientes de la bitácora asociada resueltos en este repositorio. El problema y su análisis completo están allá; aquí se anota qué se cambió en el código.

### Código

- [x] 19-10-2026 **C-01 · Quitar el `int()` de `start_offset_samples`**
  - Ahora `start_offset_samples = int(round(start_offset_time * sample_rate_hz))`, coherente con `begin_offset_time_s: 0.00005` (50 µs, 30 muestras) y `good_ramp_samples: 270` en `radar.yaml`.
  - Queda confirmarlo con una ejecución real.

- [x] 19-10-2026 **C-02 · `do_chirp` tiene `burst_slice` como principal: hacerlo como en `do_sweep`**
  - Ambas funciones copian cada chirp con `time_data[burst, self.start_offset_samples:(self.start_offset_samples + self.good_ramp_samples)] = burst_slice`.
  - Quedan 2 puntos por corregir (ver «Pendientes»).

- [x] 19-10-2026 **C-03 · Steer angle en grados, no en radianes**

- [x] 19-10-2026 **C-04 · Nuevos códigos de ganancia del ADAR1000**
  - `[11, 45, 99, 127, 127, 99, 45, 11]` en vez de `[6, 27, 66, 100, 100, 66, 27, 6]`.
  - **VGA.** Cada código fija la ganancia del VGA (*Variable Gain Amplifier*: amplificador de ganancia variable, controlado por software) de uno de los 8 elementos del arreglo. Es un valor de 7 bits, de 0 a 127.
  - **Zona recomendada.** La relación entre código y ganancia no es lineal: por debajo del código 11, la ganancia cae bruscamente. Por eso ADI recomienda usar solo los 16 dB superiores del rango (códigos ≳ 11). El código 6 que se usaba en los extremos quedaba fuera de esa zona.
  - **Curva de ganancia** (Fig. 95 del datasheet, respecto del código 127): 0 → ≈ −23 dB, 6 → −18.5 dB, 11 → ≈ −16 dB, 27 → −11 dB, 66 → −5.9 dB, 100 → −2.9 dB y 127 → 0 dB.
  - **Atenuador de 15 dB.** Además del VGA, cada canal tiene un atenuador fijo con solo dos estados: activado (−15 dB) o desactivado (0 dB). Lo comparten las rutas de recepción y de transmisión del canal. En recepción se controla con el bit 7 de los registros 0x010–0x013 (`CHx_ATTN_RX`), con lógica invertida: bit = 1 deja el atenuador en *bypass* (sin atenuación) y bit = 0 atenúa 15 dB.
  - **La Fig. 95 se midió sin el atenuador:** su leyenda indica "ATTENUATION = 1", es decir, *bypass*. Por eso el código 0 aparece en ≈ −23 dB.
  - **Comportamiento de pyadi-iio.** `set_chan_gain()` activa el atenuador automáticamente solo cuando el código es 0. En ese caso el elemento queda en ≈ −38 dB (−23 dB del VGA al mínimo, más −15 dB del atenuador), pero no se apaga. Con cualquier otro código (1–127), el atenuador queda desactivado.

  ```python
  gains = [11, 45, 99, 127, 127, 99, 45, 11]  # antes: [6, 27, 66, 100, 100, 66, 27, 6]
  ```

- [x] 19-10-2026 **C-05 · Tono de TX de 100 kHz exactos con un búfer de 600 muestras (no 6144)**
  - Búfer de TX de `N = 600` muestras a 600 kHz: 100 ciclos exactos de 100 kHz, la misma frecuencia que restan los visualizadores. El nodo avisa si el búfer no contiene un número entero de ciclos.

- [x] 19-10-2026 **C-07 · Sin bloques viejos en la cola de recepción**

  ```python
  sdr.rx_enabled_channels = [0, 1]
  sdr._rxadc.set_kernel_buffers_count(1)  # sin bloques viejos en la cola
  ```

- [x] 19-10-2026 **C-08 · Tono de TX al fondo de escala del DAC (`2**15 - 1` en vez de `2**14`)**
  - I y Q se escalan por `2**15 - 1`, porque 2¹⁵ = 32 768 no cabe en un entero de 16 bits con signo.

- [x] 19-10-2026 **C-12 · `RadarBeamform.action`: quitar la posición del pan-tilt del goal**
  - Se comentaron los campos `int32 pan_deg` e `int32 tilt_deg`: la acción no necesita recibir la posición del pan-tilt, solo saber cuándo ejecutar su rutina.

  ```text
  #int32 pan_deg
  #int32 tilt_deg
  ```

- [x] 19-10-2026 **C-13 · Dependencias de `package.xml`**
  - Se eliminó `<buildtool_depend>ament_python</buildtool_depend>`.
  - Se reemplazó `<test_depend>pytest</test_depend>` por `<test_depend>python3-pytest</test_depend>`.

- [x] 19-10-2026 **C-14 · Filtros RF del PlutoSDR fijos en 18 MHz al arranque**
  - Al arrancar se fijan `rx_rf_bandwidth` y `tx_rf_bandwidth` en 18 MHz, porque la calibración usa un filtro de 20 MHz y deja cambiada la configuración del PlutoSDR.

### Configuración (YAML y ROS 2)

- [x] 19-10-2026 **CF-02 · YAML con datos estáticos: no se pueden realizar cálculos**

- [x] 19-10-2026 **CF-03 · Archivos de calibración en ROS 2**
  - Se agregó al repositorio el archivo de calibración `channel_cal_val`.
  - Qué calibra cada archivo:
    - `channel_cal_val`: desbalance (dB) entre los dos canales Rx del PlutoSDR, uno por subarreglo.
    - `gain_cal_val`: ganancia relativa de cada elemento.
    - `phase_cal_val`: fase relativa de cada elemento, en grados.

- [x] 19-10-2026 **CF-08 · Corregir `.gitignore`**
  - Ahora solo omite los archivos de construcción del workspace y las carpetas de datos.

### Documento

- [x] 19-10-2026 **D-08 · Revisión de la edición de `README.md` y `QUICKSTART.md`**
  - Mejora de la documentación del repositorio.

### Observaciones o medio hecho

- [ ] 19-10-2026 **C-06 · Anular por completo la salida TX1**
  - En el código ya está: `radar_node.py` habilita solo Tx2 y transmite un solo arreglo. Se deja aquí, como en la bitácora asociada, hasta anotar qué falta.

  ```python
  sdr.tx_enabled_channels = [1]   # solo Tx2
  sdr.tx(iq)                      # un solo arreglo, en vez de [iq, iq]
  ```

- [ ] **C-09 · Qué lado de la FFT usar para la distancia: negativo o positivo**
  - Respondido en la sección 7.3 del documento revisado: el lado positivo, porque el eco queda en $f_c + f_b$ y el lado negativo solo tiene componentes indeseadas (imágenes y fuga de LO).
  - Falta hacer un gráfico completo y analizar con detención los reflejos en el espectro. La fuga directa de TX a RX debe aparecer cerca de +110.8 kHz (distancia ≈ 0) y no en −110.8 kHz.

- [ ] 09-11-2026 **CF-01 · Frecuencia central (IF) del PlutoSDR en 1.4 GHz en vez de 2.2 GHz**
  - **Clave del parámetro.** Se mantuvo `center_freq_hz` en `radar.yaml` y se adaptó el código (`radar_node`, `radar_visualizer` y `radar_polar_visualizer`), que declaraba `center_frequency_hz` y por eso ignoraba el valor del YAML. La bitácora asociada proponía, en cambio, cambiar la clave del YAML.
  - **Archivos modificados.** `radar.yaml` (`center_freq_hz: 1.4e9`) y `calibracion.yaml` (`cal_rx_lo_hz: 1.4e9`), además de los valores por defecto en `radar_node.py`, `calibracion_node.py` y los visualizadores. Las ventajas y desventajas quedaron comentadas en `radar.yaml` y en `radar_node.py`, junto al cálculo de `phaser.frequency`.
  - **Plan de frecuencias.** La RF no cambia (10.0–10.5 GHz), porque el LO del CN0566 se calcula como RF + IF: pasa de 12.2001–12.7001 GHz a 11.4001–11.9001 GHz.
  - **Ventajas:**
    - El LO queda dentro del rango garantizado del VCO HMC735 (10.5–12.2 GHz), con ≈ 300 MHz de margen. Con 2.2 GHz, la rampa necesitaba ≈ 13.5 V de tensión de sintonía (VTUNE), sobre los 13 V especificados, y con temperatura alta podía perder el enganche al final de cada chirp. Esto también resuelve lo planteado en D-03.
    - La calibración con la HB100 también queda dentro del rango: el barrido pide un LO de 11.4–12.1 GHz, en vez de 12.2–12.9 GHz.
    - 1.4 GHz queda en el hueco entre las bandas del GPS del Bunker (mosaic-X5 de triple banda: L5/E5/L2 ≈ 1.16–1.25 GHz y L1/E1 ≈ 1.56–1.61 GHz).
    - Los baluns de IF (TC1-1-13M+, 4.5–3000 MHz) pierden algo menos que cerca de su borde superior.
  - **Desventajas:**
    - La fuga del LO por el mezclador de TX (11.4–11.9 GHz) cae dentro de la banda de paso del filtro BFCN-1052+ (9.7–11.95 GHz) y más cerca del corte del LFCW-1062+, así que se emite con más potencia. Es una emisión fuera de banda: no genera blancos falsos, porque al mezclarse en recepción vuelve cerca de DC, fuera de la IF.
    - Las calibraciones hechas con 2.2 GHz (`channel_cal_val.pkl`, `phase_cal_val.pkl` y, por precaución, `gain_cal_val.pkl`) dejan de ser válidas, porque el desbalance entre canales y la fase entre subarreglos dependen de la IF.
    - `range_offset_hz` puede cambiar levemente por el retardo de la cadena de IF.
  - **Pendiente de verificar en hardware:** ver «CF-01 (verificación)» en «Pendientes».

  > Revisar interferencia con la banda del GPS: 1.4 GHz queda entre las bandas del GPS del robot móvil (L5/E5/L2 ≈ 1.16–1.25 GHz y L1/E1 ≈ 1.56–1.61 GHz).

### Diferencias con la bitácora asociada (cruce del 18-11-2026)

- **C-02:** allá figura como hecho; aquí quedan 2 puntos pendientes.
- **C-06:** el código ya transmite solo por Tx2, pero allá sigue en «Observaciones o medio hecho».

---

## Notas técnicas: Lo que se debe integrar con el archivo `cadena-senal-cn0566-revisado.md`

### LTE y OFDM

LTE transmite muchos tonos al mismo tiempo, cada uno en una frecuencia distinta, separados cada 15 kHz. Cada tono se llama subportadora y lleva una amplitud y una fase determinadas; esa combinación representa unos pocos bits. Un canal de 20 MHz tiene 100 bloques, es decir, 1200 tonos, que ocupan 100 × 180 kHz = 18 MHz.

OFDM es la sigla de *Orthogonal Frequency-Division Multiplexing*:

- **Multiplexing:** enviar muchas señales al mismo tiempo por un solo canal.
- **Frequency-Division:** cada señal va en una frecuencia distinta.
- **Orthogonal:** las señales no se interfieren entre sí, aunque estén muy juntas.

### Los tres PLL del sistema

Un PLL (*Phase-Locked Loop*) toma una frecuencia de referencia estable y la multiplica a otra más alta, manteniéndola enganchada a la referencia. El PlutoSDR también tiene su propio oscilador local. Cadena de relojes de recepción:

```text
oscilador de referencia del Pluto (40 MHz) × 24.576 ──► BBPLL = 983.04 MHz ÷2 ──► ADC a 491.52 MHz ──► HB3 ──► HB2 ──► HB1 ──► FIR ÷2 ──► 30.72 MHz
```

Hay tres PLL distintos, que no hay que confundir:

| PLL | Dónde está | Qué genera | Lo controla |
|---|---|---|---|
| **BBPLL** | dentro del AD9363 (PlutoSDR) | **reloj digital** (983.04 MHz) para el ADC, el DAC y los filtros; define la tasa de muestreo | `sdr.sample_rate` o el archivo `.ftr` (línea `RRX`) |
| **Sintetizadores de RF del AD9363** | dentro del AD9363 | los **osciladores locales del Pluto**, `rx_lo` y `tx_lo`: la IF de **1.4 GHz** (CF-01) | `sdr.rx_lo` y `sdr.tx_lo` (`center_freq_hz`) |
| **ADF4159 + VCO HMC735** | en la placa del CN0566 (Phaser) | el **LO del radar**, la rampa de 11.4–11.9 GHz que produce la RF de 10.0–10.5 GHz | `phaser.frequency` y `freq_dev_*` |

El BBPLL solo determina cada cuánto se toma una muestra. No tiene nada que ver con las frecuencias de radio, ni con los 1.4 GHz ni con los 10 GHz.

### Proceso de calibración paso a paso

Código: `calibracion_node.py` (orden de los pasos) y `phaser_functions.py` (mediciones). Se lanza con `ros2 launch radar_package calibracion.launch.py`, que carga `calibracion.yaml` junto con `radar.yaml`.

Esta sección responde dos preguntas que estaban en «Pendientes»:

- **¿`pcal` está en grados?** Sí. pyadi-iio suma `pcal` a la fase y aplica `% 360.0`, y el paso de fase es `phase_step_size = 2.8125` = 360/2⁶, porque el desfasador del ADAR1000 es de 6 bits. Si `pcal` estuviera en radianes, el `% 360` no tendría sentido (ver «Paso 4»).
- **¿Cómo debería ser la calibración con el modo `"tx"`?** Ver «Variante: fuente `tx`».

#### Qué se calibra y para qué

Las 8 antenas del arreglo deberían recibir con la misma ganancia y la misma fase, pero las diferencias de fabricación, cables y componentes lo impiden. Si no se corrigen, el haz apunta mal y los lóbulos laterales suben. La calibración mide esas diferencias y guarda las correcciones que después aplica `radar_node`:

| Paso | Qué mide | Archivo | Cómo lo usa el radar |
|---|---|---|---|
| `hb100` | la frecuencia exacta de la HB100 | `hb100_freq_val.pkl` | solo la usan los pasos siguientes |
| `channel` | el desbalance (dB) entre los dos canales Rx del PlutoSDR | `channel_cal_val.pkl` (`ccal`) | ajusta `rx_hardwaregain_chan0` y `rx_hardwaregain_chan1` |
| `gain` | la ganancia relativa de cada elemento | `gain_cal_val.pkl` (`gcal`) | multiplica el código del VGA de cada elemento |
| `phase` | la fase relativa de cada elemento (grados) | `phase_cal_val.pkl` (`pcal`) | se suma a la fase de cada elemento |

Montaje: la HB100 encendida, frente al arreglo, en su eje mecánico (boresight) y a ~1.5 m. No se debe ejecutar junto con `radar_node`, porque ambos configuran el mismo hardware.

#### Idea base: cómo llega la señal de la HB100 hasta Python

La señal de la HB100 baja de frecuencia en dos mezclas:

```
HB100 (f_HB) ──► mezclador del Phaser ──► IF = LO − f_HB ──► PlutoSDR ──► banda base = IF − rx_lo
                    (LO del ADF4159)                          (rx_lo = 1.4 GHz)
```

Combinando ambas: **f_banda_base = LO − f_HB − rx_lo**. Como el código elige el LO, decide dónde cae el tono en banda base:
- con LO = f_HB + rx_lo, el tono cae en 0 Hz;
- con LO = f_HB + rx_lo + 1 MHz (`cal_tone_offset_hz`), cae en **+1 MHz**: (f_HB + rx_lo + 1 MHz) − f_HB − rx_lo = +1 MHz.

Es decir, al subir el LO 1 MHz respecto del valor "justo", el tono aparece 1 MHz a la derecha de DC. Así se aleja de la **fuga del LO del PlutoSDR**, que siempre aparece en 0 Hz. Además, el código ignora ±0.5 MHz alrededor de DC (`DC_GUARD_HZ`) al buscar el tono.

#### Paso 1 · `hb100`: medir la frecuencia de la HB100

La HB100 se vende como "10.525 GHz", pero cada unidad emite algo distinto y su frecuencia varía con la temperatura. Por eso primero se mide.

**Configuración:** PlutoSDR a 30.72 MS/s (filtro `LTE20_MHz.ftr`, ventana útil de ±9.5 MHz), ganancia Rx de 0 dB, los 8 elementos con código 64, el haz al frente (fase 0) y el transmisor apagado. Cada captura tiene 1024 muestras, así que la resolución es de 30.72 MHz / 1024 = 30 kHz por bin.

**Barrido** (`find_hb100`):
1. Para cada frecuencia de prueba (`freq`), de 10.0 a 10.7 GHz en pasos de 5 MHz (140 pasos), pone el LO en `freq + rx_lo`. Con eso, una señal que estuviera exactamente en `freq` caería en 0 Hz.
2. Captura 1024 muestras y calcula su espectro. Cada punto del espectro, a una distancia f_bb de DC, corresponde a una frecuencia real **RF = freq − f_bb**. El signo menos se debe a que el LO está por encima de la señal, y el espectro queda invertido.
3. De cada captura conserva solo la zona limpia: 0.5 MHz < |f_bb| < 9.5 MHz, sin la fuga en DC ni los bordes del filtro. Cada captura ve, entonces, de freq − 9.5 a freq + 9.5 MHz, menos un hueco de ±0.5 MHz en el centro.
4. **Las capturas se traslapan.** Como el paso (5 MHz) es menor que el ancho útil a cada lado (9 MHz), las capturas vecinas se superponen, y el hueco central de una captura lo cubren sus vecinas. Por eso el paso no puede superar 9.5 − 0.5 = 9 MHz; el código lo valida.

   ```
   MHz:     9990      9995      10000     10005     10010     10015     10020
   A:     [███████████████████]░[███████████████████]
   B:               [███████████████████]░[███████████████████]
   C:                         [███████████████████]░[███████████████████]
                                 ▲         ▲         ▲
                            hueco de A  hueco de B  hueco de C
   ```

5. **Se arma un espectro completo.** Los pares (frecuencia real, nivel) de las 140 capturas se juntan en una sola lista ordenada por frecuencia, que cubre de 10.0 a 10.7 GHz sin huecos. **El punto de mayor nivel es la frecuencia de la HB100**, con una precisión de un bin (30 kHz).

**Controles antes de aceptar el resultado:**
- el máximo debe superar en al menos 15 dB a la mediana del espectro (`hb100_min_snr_db`);
- si hay otro máximo a menos de 6 dB, lejos del principal, avisa de reflejos o de otra fuente cerca;
- **prueba de contraste:** deja el tono en +1 MHz y mide su nivel con los 8 elementos encendidos (código 127) y apagados (código 0). Si viene de la HB100, debe caer al menos 15 dB (`cal_min_signal_contrast_db`). Si no cae, el máximo no entra por el arreglo: es un espurio generado dentro del propio SDR.

Resultado: `hb100_freq_val.pkl` y el gráfico `hb100_espectro`.

#### Preparación para los pasos 2–4 (`configure_cw`)

- Fija el PLL del Phaser **sin rampa**, con el LO en f_HB + rx_lo + 1 MHz: el tono queda en +1 MHz.
- Carga las calibraciones anteriores, porque los pasos dependen unos de otros, y deja los 8 elementos en código 127 con el haz al frente.
- Repite la prueba de contraste (`check_signal`) y aborta si la señal no entra por el arreglo.
- En cada medición posterior busca el **bin del tono** (`find_peak_bin`), poniendo en cero los bins de DC para no confundirlo con la fuga.

#### Paso 2 · `channel`: desbalance entre los dos canales del PlutoSDR

El arreglo tiene dos subarreglos de 4 elementos. Cada uno entra al Pluto por un canal distinto (Rx1 y Rx2), con su propio ADAR1000, mezclador, cables y entrada del Pluto.
1. Enciende solo los 4 elementos de un subarreglo (los demás en código 0) y mide el nivel del tono.
2. Repite con el otro subarreglo.
3. La diferencia en dB es `ccal`. El radar la compensa atenuando el canal más fuerte con `rx_hardwaregain`.

Va antes de `gain`, porque la medición de cada elemento pasa por uno de esos dos canales.

#### Paso 3 · `gain`: ganancia de cada elemento

1. Enciende **un elemento a la vez** (los demás en código 0) y mide el nivel del tono.
2. Toma como referencia el **más débil** y calcula `gcal[k] = nivel mínimo / nivel[k]`. El elemento más débil queda con 1.0 y los demás con valores menores.
3. El radar multiplica el código del VGA de cada elemento por su `gcal` (`set_chan_gain(..., apply_cal=True)`), para que todos aporten lo mismo.

Ojo: como `gcal` reduce los códigos, con la calibración actual los extremos del taper de C-04 (código 11) terminan escritos como 7 y 8, por debajo de la zona recomendada.

#### Paso 4 · `phase`: fase de cada elemento

Se mide por pares de elementos vecinos (0-1, 1-2, …, 6-7), buscando el **nulo** de su suma:
1. Enciende solo esos dos elementos, con la ganancia calibrada, para que tengan la misma amplitud.
2. Deja el primero en 0° y barre la fase del segundo de −180° a +180°, en pasos de 2.8125° (el paso del desfasador de 6 bits).
3. Busca el **mínimo**: ahí las dos señales están opuestas (180°) y se cancelan. Si ya estuvieran en fase, el nulo aparecería justo en 180°; la diferencia con 180° es la corrección del par.
4. Acumula las correcciones de par en par: `pcal[k+1] = pcal[k] − corrección`, con `pcal[0] = 0`.

Se busca el nulo y no el máximo porque el nulo es mucho más agudo: un error de pocos grados cambia mucho el nivel. Va después de `gain` porque el nulo solo es profundo si las dos señales tienen la misma amplitud. `pcal` queda en grados, porque pyadi-iio la suma a la fase y aplica `% 360`.

#### Resultados

- Los `.pkl` se guardan en `radar_package/resource/` (`cal_output_dir: ""`) y los gráficos en `resource/calibracion_graficos/`.
- Para que `radar_node` los use, hay que recompilar: `colcon build --packages-select radar_package`.
- Con la IF de 1.4 GHz (CF-01), el barrido de la HB100 pide un LO de 11.4–12.1 GHz y, para una HB100 en 10.525 GHz, el LO fijo de los pasos 2–4 queda en 11.926 GHz. Las calibraciones hechas con 2.2 GHz dejan de ser válidas.

#### Variante: fuente `tx` (`cal_signal_source: "tx"`)

En vez de la HB100 se usa el propio transmisor del Phaser, con una antena conectada a su salida TX y ubicada frente al arreglo:
- se omite el paso `hb100` y se usa `cal_default_signal_freq_hz` como frecuencia;
- el Pluto genera un tono de 2 MHz por Tx2 (DDS) y el Phaser lo sube a RF con el mismo LO que después usa para bajarlo;
- como el mismo LO sube y baja la señal, su efecto se cancela: el tono llega a banda base en +2 MHz, la frecuencia del DDS, sin importar el LO ni el desplazamiento de 1 MHz.
