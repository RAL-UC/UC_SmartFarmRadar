#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *
from matplotlib.widgets import Slider, RadioButtons
import os
from ament_index_python.packages import get_package_share_directory # recursos
#import threading
#from rclpy.executors import MultiThreadedExecutor
#import time

# recursos
pkg_share = get_package_share_directory('radar_package')
path_medicion_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo_centro.npy')

class RadarVisualizer(Node):
    def __init__(self):
        super().__init__('radar_visualizer')

        # suscripción a datos de radar
        # los datos son recibidos como una matriz fft de frecuencias en steering angle
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)

        self.medicion_fondo = np.load(path_medicion_fondo) # carga medicion de fondo en datos de radar

        self.filtered_data = None # data filtrada y desplazada en offset
        self.filtered_freq = None # eje x filtrado

        # parámetros configurables desde línea de comandos o launch 
        self.declare_parameter('angle_min_radar_beam', ANGLE_MIN_RADAR_BEAM) # grados
        self.declare_parameter('angle_max_radar_beam', ANGLE_MAX_RADAR_BEAM) # grados
        self.declare_parameter('angle_step_radar_beam', ANGLE_STEP_RADAR_BEAM) # grados

        # Leer parámetros
        p = self.get_parameter
        self.angle_min_radar_beam = p('angle_min_radar_beam').get_parameter_value().integer_value
        self.angle_max_radar_beam = p('angle_max_radar_beam').get_parameter_value().integer_value
        self.angle_step_radar_beam = p('angle_step_radar_beam').get_parameter_value().integer_value

        # Funciones de conversión freq <-> range (eje inferior y superior)
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + OFFSET + (d * 2 * SLOPE / C)

        # Ejes y datos que se rellenan en el primer mensaje
        self.freq = None # eje de frecuencias
        self.filtered_data = None
        self.valid_indices = None # índices >= 0 m

        self.win_funct = np.ones(GOOD_RAMP_SAMPLES, dtype=np.float64) # ventana rectangular
        #self.win_funct = np.blackman(GOOD_RAMP_SAMPLES) # ventana blackman -> posiblemente se deba considerar offset
        #self.win_funct = np.hamming(GOOD_RAMP_SAMPLES)
        self.sum_win_funct = np.sum(self.win_funct)

        # Configuración de Matplotlib interactivo
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12,7))

        # Reservar 20% del ancho izquierdo para los sliders
        plt.subplots_adjust(left=0.20, bottom=0.30, right=0.95, top=0.90)

        # FFT 
        self.line, = self.ax.plot([], [], lw=2, label='FFT')
        # Umbral CFAR
        self.line_thr, = self.ax.plot([], [], '--', lw=1.5, label='CFAR Threshold')
        # Puntos de detecciones CFAR
        self.scatter_det = self.ax.scatter([], [], s=30, c='r', marker='x', label='Detections')
        # ruido en metodo falsa alarma
        self.line_noise, = self.ax.plot([], [], ':', lw=1.5, label='Noise Variance')
        self.line_noise.set_visible(False)

        self.ax.set_xlabel("Range [m]")
        self.ax.set_ylabel("MinMax Normalized Magnitude") # normalizada min-max

        # eje secundario de rango en la parte superior
        self.secax = self.ax.secondary_xaxis('top', functions=(self.distance_to_freq, self.freq_to_distance))
        self.secax.set_xlabel("Frequency [Hz]")

        # Slider de frames (recorrido en steering angle)
        ax_slider = plt.axes([0.25, 0.05, 0.65, 0.03])
        init_angle = np.clip(0, self.angle_min_radar_beam, self.angle_max_radar_beam) if self.angle_min_radar_beam <= 0 <= self.angle_max_radar_beam else self.angle_min_radar_beam
        self.sld_angle = Slider(ax_slider, 'Steering angle', self.angle_min_radar_beam, self.angle_max_radar_beam, valinit=init_angle, valstep=self.angle_step_radar_beam)

        # CONTROLES INTERACTIVOS
        # Slider para num_guard_cells
        ax_guard = plt.axes([0.02, 0.30, 0.015, 0.60])
        self.sld_guard = Slider(ax_guard, 'Guard', 1, 30, valinit=20, valstep=1, orientation='vertical')

        ax_ref = plt.axes([0.05, 0.30, 0.015, 0.60])
        self.sld_ref = Slider(ax_ref, 'Ref', 1, 70, valinit=50, valstep=1, orientation='vertical')

        ax_bias = plt.axes([0.08, 0.30, 0.015, 0.60])
        self.sld_bias = Slider(ax_bias, 'Bias', 0.0, 30.0, valinit=12, valstep=1, orientation='vertical')

        # Slider para fa_rate (solo para método false_alarm)
        ax_fa = plt.axes([0.11, 0.30, 0.015, 0.60]) 
        self.sld_fa = Slider(ax_fa, 'FA\nRate', 0.0, 2.0, valinit=0.5, valstep=0.01, orientation='vertical')
        self.sld_fa.ax.set_visible(False)

        # RadioButtons para método CFAR
        ax_method = plt.axes([0.01, 0.02, 0.12, 0.15])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        #self.legend = self.ax.legend(loc='upper right')

        #self._mpl_timer = None
        #self._pending_idx = 0

        self.sld_angle.on_changed(self.on_slider_change)
        # disparo de actualización
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias, self.sld_fa):
            ctl.on_changed(lambda v: self.update_display(int(self.sld_angle.val)))
        self.radio_method.on_clicked(lambda label: self.update_display(int(self.sld_angle.val)))

        # mostrar figura
        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    def update_display(self, angle_val: int):
        """Dibuja FFT + CFAR solamente en los índices válidos"""
        idx = self.ang_to_idx(angle_val)
        mag = self.filtered_data[idx, :]

        #mag_min = np.min(mag)
        #mag_max = np.max(mag)
        #
        #if mag_max > mag_min:
        #    mag = (mag - mag_min) / (mag_max - mag_min) # normalizacion min-max
        #else:
        #    mag = np.zeros_like(mag)

        # valores de los controles de interfaz
        ng = int(self.sld_guard.val) # celdas de guarda
        nr = int(self.sld_ref.val) # celdas de referencia
        b = float(self.sld_bias.val) # valor bias
        m = self.radio_method.value_selected # metodo de calculo del umbral
        fa_rate = float(self.sld_fa.val)

        total_ext = ng + nr
        mag_ext= self.extend_with_means(mag, total_ext)

        # CFAR sobre mag filtrada
        if m == "false_alarm":
            thresh, targets, noise_variance = cfar(mag_ext, num_guard_cells=ng, num_ref_cells=nr, bias=b, cfar_method=m, fa_rate=fa_rate)
            thresh = self.unpad(thresh, total_ext)
            targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
            noise_line = np.ones_like(mag) * noise_variance
            self.line_noise.set_data(self.freq, noise_line)
            self.line_noise.set_visible(True)
            self.sld_fa.ax.set_visible(True)
        else:
            thresh, targets = cfar(mag_ext, num_guard_cells=ng, num_ref_cells=nr, bias=b, cfar_method=m)
            thresh = self.unpad(thresh, total_ext)
            targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
            self.line_noise.set_visible(False)
            self.sld_fa.ax.set_visible(False)

        # Forzar actualización manual de leyenda con elementos visibles
        handles, labels = [], []
        for obj in [self.line, self.line_thr, self.scatter_det, self.line_noise]:
            if obj.get_visible() and obj.get_label() != '_nolegend_':
                handles.append(obj)
                labels.append(obj.get_label())
        self.ax.legend(handles, labels, loc='upper right')

        # Detectar los enmascarados (masked = True)
        det_indices = np.where(targets.mask)[0] # obtener valores objetivos
        # Detectar los no enmascarados (masked = False)
        #det_indices = np.where(~targets.mask)[0] # obtener valores objetivos

        x = self.freq_to_distance(self.freq)

        # actualizar líneas y puntos
        self.line.set_data(x, mag)
        self.line_thr.set_data(x, thresh)
        self.scatter_det.set_offsets(np.c_[x[det_indices], mag[det_indices]])
        # Ajustar límites de X e Y
        # X: distancia válida
        self.ax.set_xlim(x[0], x[-1])
        # Y: 0 a 1 (normalizado min-max)
        #self.ax.set_ylim(np.min(mag), np.max(mag)) # np.min(mag), np.max(mag)
        #self.ax.set_ylim(0, 1)
        self.ax.set_ylim(-80, 50)
        self.secax.set_xlim(self.freq[0], self.freq[-1])

        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')
        # Reconstruir matriz original
        #arr = np.array(msg.data, dtype=msg.dtype) # arreglo vectorial
        #self.get_logger().info(f"mat.dtype {msg.data_real}")

        n_steering_angle, n_bins = [msg.rows, msg.cols]
        #mat = arr.reshape((n_steering_angle, n_bins)) # arreglo matricial (n_steering_angle, n_bins)
        data_real = np.array(msg.data_real,  dtype=msg.dtype)
        data_real = data_real.reshape((n_steering_angle, n_bins))
        data_imag = np.array(msg.data_imag,  dtype=msg.dtype)
        data_imag = data_imag.reshape((n_steering_angle, n_bins))


        mat = data_real + 1j*data_imag
        mat = mat.reshape((n_steering_angle, n_bins))
        mat[:,:GOOD_RAMP_SAMPLES] = mat[:,:GOOD_RAMP_SAMPLES] * self.win_funct
        sp = np.fft.fftshift(np.abs(np.fft.fft(mat, axis=1)), axes=1)
        s_mag = sp / self.sum_win_funct
        s_mag = np.maximum(s_mag, 10 ** (-15))
        mat = 20 * np.log10(s_mag / (2 ** 11)) # s_dbfs
        

        #mat = mat[:161, :]
        #self.get_logger().info(f"{self.medicion_fondo.shape} | {mat.shape}")
        #if self.medicion_fondo is not None:
        #    if self.medicion_fondo.shape == mat.shape:
        #        mat = mat - self.medicion_fondo
        #        #mat = self.medicion_fondo
        #    elif self.medicion_fondo.shape[1] == mat.shape[1] and mat.shape[0] == 1:
        #        fondo_1x = np.array(self.medicion_fondo[80]).reshape(n_steering_angle, n_bins) # 1×N
        #        mat = mat - fondo_1x
        #        #mat = fondo_1x
        #        self.get_logger().info(f"{mat.shape}")
        #    else:
        #        self.get_logger().warn(
        #            f"Shape fondo {self.medicion_fondo.shape} != datos {mat.shape}; omitiendo resta."
        #        )

        # Construir eje de frecuencia completo y corrimiento
        freq = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, n_bins, endpoint=False)
        distance = self.freq_to_distance(freq)
        # filtrar solo distancias >= 0
        self.valid_indices = np.where(distance >= 0)[0]
        self.filtered_data = mat[:, self.valid_indices]
        # atenuar valores iniciales
        #row_means = np.mean(self.filtered_data, axis=1)
        #self.filtered_data[:,:IDX_ATTENUATION] = row_means[:, np.newaxis]

        rows = self.filtered_data.shape[0]
        if rows == 1:
            self.sld_angle.ax.set_visible(False) # sin control de ángulo
        else:
            self.sld_angle.ax.set_visible(True)
        
        self.freq = freq[self.valid_indices]
        # Actualizar slider sin mover thumb
        #self.sld_angle.valmax = n_steering_angle - 1
        #self.sld_angle.ax.set_xlim(self.sld_angle.valmin, self.sld_angle.valmax)

        # Redibujar en la posición actual del slider
        angle_val = self.sld_angle.val
        #angle_val = int(max(self.sld_angle.valmin, min(angle_val, self.sld_angle.valmax)))
        self.update_display(angle_val)

    def on_slider_change(self, val: float):
        angle_val = int(val)
        
        if self.filtered_data is not None:
            self.update_display(angle_val)

    def ang_to_idx(self, ang: int) -> int:
        """Convierte un ángulo (deg) a índice de fila [0..rows-1]."""
        if self.filtered_data is None:
            return 0
        rows = self.filtered_data.shape[0]
        # redondea al múltiplo de step
        idx = int(round((ang - self.angle_min_radar_beam) / float(self.angle_step_radar_beam)))
        return max(0, min(rows - 1, idx))

    def extend_with_means(self, mag, total_guard_ref):
        """
        Extiende el vector mag agregando `total_guard_ref` celdas al inicio y al final,
        usando el promedio de las primeras y últimas `total_guard_ref` celdas reales
        """
        mean_start = np.mean(mag[:total_guard_ref])
        mean_end = np.mean(mag[-total_guard_ref:])

        # Relleno
        pad_start = np.full(total_guard_ref, mean_start)
        pad_end = np.full(total_guard_ref, mean_end)

        # Vector extendido
        mag_ext = np.concatenate([pad_start, mag, pad_end])

        return mag_ext

    # Función para recortar de vuelta
    def unpad(self, v, total_guard_ref):
        return v[total_guard_ref:-total_guard_ref]


def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
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
