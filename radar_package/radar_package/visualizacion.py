#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *
from matplotlib.widgets import Slider, RadioButtons

# debo mejorar recurso de datos al infinito
path_base_data = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/infinito1.npy"

class RadarVisualizer(Node):
    def __init__(self):
        super().__init__('radar_visualizer')

        # suscripción a datos de radar
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)
        # los datos son recibidos como una matriz fft de frecuencias en steering angle

        self.base_data = np.load(path_base_data) # carga de banda base en datos de radar

        self.filtered_data = None # data filtrada y desplazada en offset
        self.filtered_freq = None # eje x filtrado

        # parámetros configurables desde línea de comandos o launch 
        self.declare_parameter('angle_min', -80) # grados
        self.declare_parameter('angle_max', 80) # grados
        self.declare_parameter('angle_step', 1) # grados

        # Leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        # Funciones de conversión freq <-> range (eje inferior y superior)
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + (d * 2 * SLOPE / C)

        # Ejes y datos que se rellenan en el primer mensaje
        self.freq = None # eje de frecuencias
        self.freq_offset_index = None # corrimiento en bins
        self.valid_indices = None # índices >= 0 m

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
        #self.ax.legend(loc='upper right')

        # eje secundario de rango en la parte superior
        self.secax = self.ax.secondary_xaxis('top', functions=(self.distance_to_freq, self.freq_to_distance))
        self.secax.set_xlabel("Frequency [Hz]")

        # Slider de frames (recorrido en steering angle)
        ax_slider = plt.axes([0.25, 0.05, 0.65, 0.03])
        self.slider = Slider(ax_slider, 'Steering angle', self.angle_min, self.angle_max, valinit=0, valstep=self.angle_step)
        self.slider.on_changed(self.on_slider_change)

        # CONTROLES INTERACTIVOS
        # Slider para num_guard_cells
        ax_guard = plt.axes([0.02, 0.30, 0.015, 0.60])
        self.sld_guard = Slider(ax_guard, 'Guard', 1, 20, valinit=5, valstep=1, orientation='vertical')

        ax_ref = plt.axes([0.05, 0.30, 0.015, 0.60])
        self.sld_ref = Slider(ax_ref, 'Ref', 1, 50, valinit=15, valstep=1, orientation='vertical')

        ax_bias = plt.axes([0.08, 0.30, 0.015, 0.60])
        self.sld_bias = Slider(ax_bias, 'Bias', 0.0, 1.0, valinit=0.1, valstep=0.01, orientation='vertical')

        # Slider para fa_rate (solo para método false_alarm)
        ax_fa = plt.axes([0.11, 0.30, 0.015, 0.60]) 
        self.sld_fa = Slider(ax_fa, 'FA\nRate', 0.0, 2.0, valinit=0.5, valstep=0.01, orientation='vertical')
        self.sld_fa.ax.set_visible(False)

        # RadioButtons para método CFAR
        ax_method = plt.axes([0.01, 0.02, 0.12, 0.15])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        # disparo de actualización
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias, self.sld_fa):
            ctl.on_changed(lambda v: self.update_display(int(self.slider.val)))
        self.radio_method.on_clicked(lambda label: self.update_display(int(self.slider.val)))

        # mostrar figura
        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    def update_display(self, idx: int):
        """Dibuja FFT + CFAR solamente en los índices válidos"""
        mag = self.filtered_data[idx, :]

        arr_min = np.min(mag)
        arr_max = np.max(mag)

        if arr_max > arr_min:
            mag = (mag - arr_min) / (arr_max - arr_min) # normalizacion min-max
        else:
            mag = np.zeros_like(mag)

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

        det_indices = np.where(targets.mask)[0] # obtener valores objetivos

        x = self.freq_to_distance(self.freq)

        # actualizar líneas y puntos
        self.line.set_data(x, mag)
        self.line_thr.set_data(x, thresh)
        self.scatter_det.set_offsets(np.c_[x[det_indices], mag[det_indices]])
        # Ajustar límites de X e Y
        # X: distancia válida
        self.ax.set_xlim(x[0], x[-1])
        # Y: 0 a 1 (normalizado min-max)
        self.ax.set_ylim(0, 1) # np.min(mag), np.max(mag)
        self.secax.set_xlim(self.freq[0], self.freq[-1])

        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        # Reconstruir matriz original
        arr = np.array(msg.data, dtype=msg.dtype) # arreglo vectorial
        n_steering_angle, n_bins = [msg.rows, msg.cols]
        mat = arr.reshape((n_steering_angle, n_bins)) # arreglo matricial (n_steering_angle, n_bins)
        #mat = mat - self.base_data # banda base

        # Construir eje de frecuencia completo y corrimiento
        freq = np.linspace(-SAMPLE_RATE/2, SAMPLE_RATE/2, n_bins, endpoint=False)
        freq_step = freq[1] - freq[0] # SAMPLE_RATE/n_bins
        self.freq_offset_index = int(OFFSET / freq_step)

        distance = self.freq_to_distance(freq)

        # Filtrar solo distancias >= 0
        self.valid_indices = np.where(distance >= 0)[0]
        
        # Desplazar datos en frecuencia y luego filtrar columnas
        rolled = np.roll(mat, -self.freq_offset_index, axis=1)
        self.filtered_data = rolled[:, self.valid_indices]
        
        self.freq = freq[self.valid_indices]
        # Actualizar slider sin mover thumb
        #self.slider.valmax = n_steering_angle - 1
        #self.slider.ax.set_xlim(self.slider.valmin, self.slider.valmax)

        # Redibujar en la posición actual del slider
        angle = self.slider.val
        idx = int(max(self.slider.valmin, min(angle, self.slider.valmax)))
        self.update_display(idx)

    def on_slider_change(self, val: float):
        idx = int(val)
        if self.filtered_data is not None:
            self.update_display(idx)

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
