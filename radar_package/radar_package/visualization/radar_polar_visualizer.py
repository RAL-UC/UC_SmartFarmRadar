#!/usr/bin/env python3
import rclpy # ros2
from rclpy.node import Node # clase nodo de ros2
import numpy as np # calculo matematico
import matplotlib.pyplot as plt # graficos
from matplotlib.widgets import Slider, RadioButtons, CheckButtons # interfaz de usuario
from radar_msg.msg import RadarData # mensaje de radar
from radar_package.processing.target_detection_dbfs import cfar, cfar_adaptive_edge # objetivos de deteccion
import os # sistema operativo
from ament_index_python.packages import get_package_share_directory  # recursos

# recursos
#pkg_share = get_package_share_directory('radar_package')
# filtro resta de fondo
#path_medicion_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo_centro2.npy')


class RadarPolarVisualizer(Node):
    """
    Visualizador tipo PPI (Plan Position Indicator): representa en coordenadas
    polares la magnitud (dBfs) de todo el barrido de steering angles vs. distancia,
    con las detecciones CFAR superpuestas como puntos.
    """

    def __init__(self):
        super().__init__('radar_polar_visualizer')

        # parámetros configurables desde línea de comandos o archivo de configuracion
        # radar.yaml
        self.declare_parameter('element_spacing_m', 0.014)
        self.declare_parameter('sample_rate_hz', 0.6e6)
        self.declare_parameter('center_frequency_hz', 2.2e9)
        self.declare_parameter('signal_freq_hz', 100e3)
        self.declare_parameter('slope_hz_per_s', 1.0e12)
        self.declare_parameter('range_offset_hz', 10760.0)
        self.declare_parameter('speed_of_light', 3e8)
        self.declare_parameter('good_ramp_samples', 270)
        self.declare_parameter('rbeam_angledeg_min', -80)
        self.declare_parameter('rbeam_angledeg_max', 80)
        self.declare_parameter('rbeam_angledeg_step', 1)

        # processing.yaml
        self.declare_parameter('cfar_guard_cells', 15)
        self.declare_parameter('cfar_reference_cells', 45)
        self.declare_parameter('cfar_bias', 10)
        self.declare_parameter('range_filter_min_m', 0.0)
        self.declare_parameter('range_filter_max_m', 5.0)

        # carga de valores
        # radar.yaml
        self.element_spacing_m = self.get_parameter('element_spacing_m').value
        self.sample_rate_hz = self.get_parameter('sample_rate_hz').value
        self.center_frequency_hz = self.get_parameter('center_frequency_hz').value
        self.signal_freq_hz = self.get_parameter('signal_freq_hz').value
        self.slope_hz_per_s = self.get_parameter('slope_hz_per_s').value
        self.range_offset_hz = self.get_parameter('range_offset_hz').value
        self.speed_of_light = self.get_parameter('speed_of_light').value
        self.good_ramp_samples = self.get_parameter('good_ramp_samples').value
        self.rbeam_angledeg_min = self.get_parameter('rbeam_angledeg_min').value
        self.rbeam_angledeg_max = self.get_parameter('rbeam_angledeg_max').value
        self.rbeam_angledeg_step = self.get_parameter('rbeam_angledeg_step').value

        # processing.yaml
        self.cfar_guard_cells = self.get_parameter('cfar_guard_cells').value
        self.cfar_reference_cells = self.get_parameter('cfar_reference_cells').value
        self.cfar_bias = self.get_parameter('cfar_bias').value
        self.range_filter_min_m = self.get_parameter('range_filter_min_m').value
        self.range_filter_max_m = self.get_parameter('range_filter_max_m').value

        # suscripción a datos de radar
        # los datos son recibidos como una matriz fft de frecuencias en steering angle
        # 10 representa la profundidad de la cola de mensajes
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)

        #self.medicion_fondo = np.load(path_medicion_fondo)  # carga medicion de fondo en datos de radar

        # Funciones de conversión freq <-> range
        self.freq_to_distance = lambda f: (f - self.signal_freq_hz - self.range_offset_hz) * self.speed_of_light / (2 * self.slope_hz_per_s)
        self.distance_to_freq = lambda d: self.signal_freq_hz + self.range_offset_hz + (d * 2 * self.slope_hz_per_s / self.speed_of_light)

        # datos que se rellenan en el primer mensaje
        self.filtered_data = None  # data magnitud filtrada "eje y" desplazada en offset (dBfs)
        #self.filtered_phase = None # fase filtada "eje y" desplazada en offset (deg)
        self.filtered_freq = None # # eje de frecuencias filtrado "eje x" (Hz)
        self.freq = None # eje de frecuencias
        self.distance = None # eje de distancias
        self.angles_deg = None # steering angles en grados (uno por fila de filtered_data)
        self.valid_indices = None # índices de distancia válidos >= 0 m

        # ventana aplicada antes de la FFT
        self.win_funct = np.ones(self.good_ramp_samples, dtype=np.float64)
        self.sum_win_funct = np.sum(self.win_funct)

        # Configuración de Matplotlib interactivo
        plt.ion()
        self.fig = plt.figure(figsize=(10, 9)) # (ancho, alto) en pulgadas
        self.ax = self.fig.add_subplot(111, projection='polar') # lienzo de ejes polares $(theta, r)$  (1 fila, 1 columna, 1º gráfico)
        self.ax.set_theta_zero_location('N') # 0° apuntando "hacia adelante" Norte o parte superior
        self.ax.set_theta_direction(-1) # ángulos positivos hacia la derecha
        self.ax.set_thetamin(self.rbeam_angledeg_min) # límite angular izquierdo
        self.ax.set_thetamax(self.rbeam_angledeg_max) # límite angular derecho
        self.ax.set_title("PPI - Barrido de radar (steering angle vs. distancia)", pad=20) # titulo y distancia en puntos entre el título y el borde superior del gráfico
        self.ax.set_rlabel_position(135) # angulo al que aparece el eje numérico de lecturas de distancia (r)

        self.mesh = None # malla de angulo deg y magnitud dBfs
        self.cbar = None # colorbar de magnitud
        # dibujar dinámicamente los objetivos detectados por el algoritmo CFAR.
        self.scatter_det = self.ax.scatter([], [], s=25, c='red', marker='x', label='Detecciones CFAR', zorder=5) 

        # Reservar espacio inferior panel CFAR e izquierdo para escala de color
        plt.subplots_adjust(left=0.18, bottom=0.32, right=0.90, top=0.90)

        # CONTROLES INTERACTIVOS (escala de color, vmin/vmax)
        # rango amplio para cubrir distintas configuraciones de ganancia
        ax_vmin = plt.axes([0.03, 0.32, 0.015, 0.58])
        self.sld_vmin = Slider(ax_vmin, "vmin\n(dBfs)", -150.0, 50.0, valinit=-80.0, valstep=1.0,
                                orientation='vertical')

        ax_vmax = plt.axes([0.08, 0.32, 0.015, 0.58])
        self.sld_vmax = Slider(ax_vmax, "vmax\n(dBfs)", -150.0, 150.0, valinit=50.0, valstep=1.0,
                                orientation='vertical')

        # CONTROLES INTERACTIVOS (CFAR)
        ax_guard = plt.axes([0.10, 0.20, 0.32, 0.02])
        self.sld_guard = Slider(ax_guard, "Guard (N)", 1, 30, valinit=self.cfar_guard_cells, valstep=1)

        ax_ref = plt.axes([0.10, 0.16, 0.32, 0.02])
        self.sld_ref = Slider(ax_ref, "Ref (N)", 1, 70, valinit=self.cfar_reference_cells, valstep=1)

        ax_bias = plt.axes([0.10, 0.12, 0.32, 0.02])
        self.sld_bias = Slider(ax_bias, "Bias (dBfs)", 0.0, 30.0, valinit=self.cfar_bias, valstep=1)

        ax_fa = plt.axes([0.10, 0.08, 0.32, 0.02])
        self.sld_fa = Slider(ax_fa, "FA Rate", 0.0, 2.0, valinit=0.5, valstep=0.01)
        self.sld_fa.ax.set_visible(False)

        ax_method = plt.axes([0.52, 0.04, 0.16, 0.20])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        ax_check = plt.axes([0.72, 0.10, 0.20, 0.10])
        self.chk_det = CheckButtons(ax_check, ['Mostrar detecciones'], [True])

        # callbacks del panel de control
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias, self.sld_fa,
                    self.sld_vmin, self.sld_vmax):
            ctl.on_changed(lambda v: self.update_display()) # detecta el valor que a cambiado para actualizar la interfaz
        self.radio_method.on_clicked(lambda label: self.update_display()) # pasa el texto de la opción seleccionada
        self.chk_det.on_clicked(lambda label: self.update_display()) # pasa el texto de la opción seleccionada

        plt.show(block=False) # despliegue de ventana
        # temporizador que fuerza el refresco de la interfaz
        # Intervalo de tiempo en segundos
        # "respiro" programado que conecta los dos motores de tu aplicación: el bucle de eventos de ROS 2 y la interfaz gráfica de Matplotlib
        self.create_timer(0.05, lambda: plt.pause(0.001))

    def _set_fa_visible(self, visible: bool):
        # mostrar/ocultar slider de fa_rate según método CFAR seleccionado
        self.sld_fa.ax.set_visible(bool(visible))
        # GUI: Graphical User Interface o Interfaz Gráfica de Usuario
        self.fig.canvas.draw_idle() # dibujar en la GUI

    #def extend_with_means(self, mag, total_guard_ref):
    #    """Extiende el vector con el promedio de sus extremos, para evitar bordes en el CFAR."""
    #    mean_start = np.mean(mag[:total_guard_ref])
    #    mean_end = np.mean(mag[-total_guard_ref:])
    #    pad_start = np.full(total_guard_ref, mean_start)
    #    pad_end = np.full(total_guard_ref, mean_end)
    #    return np.concatenate([pad_start, mag, pad_end])
    #
    #def unpad(self, v, total_guard_ref):
    #    return v[total_guard_ref:-total_guard_ref]

    def compute_cfar_mask(self, mat):
        """Aplica CFAR fila por fila (un ángulo de steering a la vez) y devuelve
        una máscara booleana (angulos x rango) con las detecciones."""
        ng = int(self.sld_guard.val) # numero celdas de guarda
        nr = int(self.sld_ref.val) # numero celdas de referencia
        b = float(self.sld_bias.val) # valor base
        m = self.radio_method.value_selected # metodo seleccionado
        # probabilidad de detectar un blanco cuando en realidad solo hay ruido
        fa_rate = float(self.sld_fa.val)
        #total_ext = ng + nr # suma total de la extension de celdas de guarda y referencia

        rows, cols = mat.shape # tamaño para defenir la matriz de enmascaramiento de detecciones
        det_mask = np.zeros((rows, cols), dtype=bool) # mascara de detecciones

        use_fa = (m == "false_alarm") # si el metodo false_alarm fue seleccionado
        self._set_fa_visible(use_fa) # visibilizar el slide en el panel de control

        #self.get_logger().info(f"tamaño: {mat.shape}")
        # por cada fila
        for i in range(rows):
            #mag_ext = self.extend_with_means(mat[i, :], total_ext)
            if use_fa: # si metodo de probabilidad de falsa alarma esta activo
                _, targets, _ = cfar_adaptive_edge(mat[i,:], num_guard_cells=ng, num_ref_cells=nr,
                                      bias=b, cfar_method=m, fa_rate=fa_rate, logger=self.get_logger())
            else:
                _, targets = cfar_adaptive_edge(mat[i,:], num_guard_cells=ng, num_ref_cells=nr,
                                   bias=b, cfar_method=m, logger=self.get_logger())
            #targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
            # oculta las posiciones donde la señal supera el umbral CFAR
            # detectar los enmascarados (masked = True)
            det_mask[i, :] = targets.mask

        return det_mask

    def update_display(self):
        """Redibuja el diagrama polar completo (magnitud + detecciones CFAR)."""
        if self.filtered_data is None or self.freq is None:
            return

        # indexing controla cómo se orientan las dimensiones (filas y columnas)
        THETA, R = np.meshgrid(self.angles_rad, self.distance, indexing='ij')

        # límites de la escala de color, tomados de los sliders vmin/vmax
        # ajustes de magnitudes para visualizacion
        vmin = float(self.sld_vmin.val)
        vmax = float(self.sld_vmax.val)
        if vmin >= vmax:
            # evita un rango inválido mientras el usuario ajusta las barras
            vmax = vmin + 1.0

        # se borra el mapa anterior para dibujar el nuevo
        if self.mesh is not None:
            self.mesh.remove()

        # self.filtered_data # (angulo x magnitud del rango) en dBfs
        self.mesh = self.ax.pcolormesh(THETA, R, self.filtered_data, shading='auto', cmap='viridis',
                                        vmin=vmin, vmax=vmax)

        # barra de color de escala cromatica
        if self.cbar is None:
            self.cbar = self.fig.colorbar(self.mesh, ax=self.ax, pad=0.12, shrink=0.8)
            self.cbar.set_label("Magnitud (dBfs)")
        else:
            self.cbar.update_normal(self.mesh)

        show_det = self.chk_det.get_status()[0] # Consulta el estado de un botón o casilla gráfica
        # visualizar detecciones CFAR
        if show_det:
            det_mask = self.compute_cfar_mask(self.filtered_data)
            det_rows, det_cols = np.where(det_mask)
            theta_det = self.angles_rad[det_rows]
            r_det = self.distance[det_cols]
            # une los arreglos en pares de coordenadas
            # actualiza la posición de las marcas (scatter plot) en el gráfico sin tener que destruirlo y recrearlo
            self.scatter_det.set_offsets(np.c_[theta_det, r_det])
            self.scatter_det.set_visible(True)
        else:
            self.scatter_det.set_visible(False)

        # Establece el radio máximo del gráfico polar según el último valor de distancia
        # Si la distancia es 0 o inválida: Asigna un radio por defecto de 1 metro
        self.ax.set_rmax(self.distance[-1] if self.distance[-1] > 0 else 1.0)
        # Ubica la leyenda fuera del área circular del radar
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.1))
        # Solicita a Matplotlib que redibuje el lienzo en el próximo ciclo de inactividad de la interfaz.
        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        #self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id={msg.header.frame_id}')
        # mensaje informativo header
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
        # float64: 8 bytes (64 bits) de memoria para representar números reales positivos o negativos con decimales
        # admite valores desde aprox -1.7 x10**(308) a 1.7 x10**(308)
        try:
            np_dtype = np.dtype(msg.dtype) # tipo de dato de radar float64
        except Exception:
            np_dtype = np.float64

        rows, cols = [msg.rows, msg.cols]

        # separacion de datos reales e imaginarios
        data_real = np.asarray(msg.data_real, dtype=np_dtype).reshape((rows, cols))
        data_imag = np.asarray(msg.data_imag, dtype=np_dtype).reshape((rows, cols))

        #mat_mag = np.sqrt(data_real**2 + data_imag**2) # magnitud

        # NO usar arctan(Q/I) — pierde el cuadrante y falla si I=0
        # arctan2 devuelve el ángulo correcto en [-pi, pi]
        #mat_ph = np.arctan2(Q, I) 

        # data type complex128
        mat = data_real + 1j * data_imag
        mat[:, :self.good_ramp_samples] *= self.win_funct[None, :]
        # multiplicar por la ventana en el dominio del tiempo, convolucion en el dominio de la frecuencia
        # implicancias principales: ancho del lobulo principal y altura de los lobulos secundarios

        
        sp = np.fft.fftshift(np.fft.fft(mat, axis=1), axes=1) # fft y shift a centro
        # calculo del valor absoluto y normalización por la suma de los coeficientes de la ventana
        s_mag = np.abs(sp) / self.sum_win_funct
        # # pone un piso minimo para evitar valores pequeños en s_mag y posteriormente -inf con el logaritmo
        s_mag = np.maximum(s_mag, 10 ** (-15))
        # normalizando respecto al valor máximo posible del ADC (full-scale) 12 bits con signo
        # Por convención, la magnitud en decibeles de una señal se calcula como
        # 20 * log_10 (A/A_ref)
        # Si un bin de FFT tiene s_mag = 2048 -> 0dBFS
        # decibeles referidos al máximo teórico de la ADC
        mat = 20 * np.log10(s_mag / (2 ** 11)) # Decibelios respecto a la Escala Completa
        # se obtiene un rango desde -366.22 dbfs a 0 aprox

        # filtro de medición de fondo
        #if self.medicion_fondo is not None:
        #    if self.medicion_fondo.shape == mat.shape:
        #        mat = mat - self.medicion_fondo
        #    elif self.medicion_fondo.shape[1] == mat.shape[1] and mat.shape[0] == 1:
        #        fondo_1x = np.array(self.medicion_fondo[80]).reshape(rows, cols)
        #        mat = mat - fondo_1x
        #    else:
        #        self.get_logger().warn(
        #            f"Shape fondo {self.medicion_fondo.shape} != datos {mat.shape}; omitiendo resta."
        #        )

        # eje de frecuencia y filtro por distancia válida
        freq = np.linspace(-self.sample_rate_hz / 2, self.sample_rate_hz / 2, cols, endpoint=False) # eje de frecuencias simetrico
        # se considera el offset en la conversion
        # si la señal es de 100 kHz y hay un offset de 10 kHz
        # si la señal tiene una frecuencia base de 100 kHz
        # se traduce en una distancia de -1.5m
        # ese valor en el filtro resultaria en false y se omite
        distance = self.freq_to_distance(freq) # transformacion de freq a distancia considerando offset
        self.valid_indices = np.where((distance >= self.range_filter_min_m) & (distance <= self.range_filter_max_m))[0]
        # para todos los angulos se aplica el filtro al igual que el eje de frecuencias
        self.filtered_data = mat[:, self.valid_indices]
        self.freq = freq[self.valid_indices]
        self.distance = self.freq_to_distance(self.freq)

        # eje angular: un steering angle por fila de la matriz
        self.angles_deg = self.rbeam_angledeg_min + np.arange(rows) * self.rbeam_angledeg_step
        self.angles_rad = np.deg2rad(self.angles_deg)

        self.update_display()


def main(args=None):
    rclpy.init(args=args)
    node = RadarPolarVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo interrumpido por el usuario.")
    except Exception as e:
        print(f"Excepción no controlada: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()