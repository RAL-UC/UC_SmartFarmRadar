#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons
from radar_msg.msg import RadarData    # Ajusta al nombre real de tu paquete
from radar_package.target_detection_dbfs import cfar                 # Ajusta a la ruta real de tu implementación CFAR
import radar_package.parametros as cfgr

path_base_data = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/infinito1.npy"
class RadarVisualizer(Node):
    def __init__(self):
        super().__init__('radar_visualizer')

        # Suscripción
        self.subscription = self.create_subscription(
            RadarData,
            'radar_data',
            self.listener_callback,
            10
        )

        # ----------------------------------------------------
        # Parámetros de radar / FFT / rango
        # ----------------------------------------------------
        self.sample_rate = cfgr.sample_rate # Hz
        self.signal_freq = cfgr.center_freq # Hz
        self.offset = 10.76e3       # Hz
        self.c = 3e8                # m/s
        self.BW = 500e6             # Hz
        self.ramp_time = 500e-6     # s
        self.slope = self.BW / self.ramp_time

        self.base_data = np.load(path_base_data) 

        self.filtered_fft_data = None     # data filtrada y desplazada
        self.freq = None                  # eje x filtrado
        # ----------------------------------------------------
        # Funciones de conversión freq <-> range para el eje superior
        # ----------------------------------------------------
        self.freq_to_distance = lambda f: (f - self.signal_freq) * self.c / (2 * self.slope)
        self.distance_to_freq = lambda d: self.signal_freq + (d * 2 * self.slope / self.c)

        # Ejes y datos que se rellenan en el primer mensaje
        self.raw_freq = None              # frecuencia completa
        self.freq_offset_index = None     # corrimiento en bins
        self.range_axis = None            # distancias completas
        self.valid_indices = None         # índices >= 0 m

        # ----------------------------------------------------
        # Configuración de Matplotlib interactivo
        # ----------------------------------------------------
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10,5))

        # Reservar 20% del ancho izquierdo para los sliders
        plt.subplots_adjust(left=0.20, bottom=0.30, right=0.95, top=0.95)

        # 1) FFT original
        self.line, = self.ax.plot([], [], lw=2, label='FFT')
        # 2) Umbral CFAR
        self.line_thr, = self.ax.plot([], [], '--', lw=1.5, label='CFAR Thresh')
        # 3) Detecciones CFAR
        self.scatter_det = self.ax.scatter([], [], s=30, c='r', marker='x', label='Detections')

        self.ax.set_xlabel("Frequency [Hz]")
        self.ax.set_ylabel("Magnitude")
        self.ax.legend(loc='upper right')

        # Eje secundario de rango
        self.secax = self.ax.secondary_xaxis(
            'top',
            functions=(self.freq_to_distance, self.distance_to_freq)
        )
        self.secax.set_xlabel("Range [m]")

        # Slider de frames
        ax_slider = plt.axes([0.25, 0.1, 0.65, 0.03])
        self.slider = Slider(ax_slider, 'Frame', 0, 0, valinit=0, valstep=1)
        self.slider.on_changed(self.on_slider_change)

        # ——————————————————————————————
        #  NUEVOS CONTROLES INTERACTIVOS
        # ——————————————————————————————
        # Slider para num_guard_cells
        ax_guard = plt.axes([0.02, 0.30, 0.015, 0.60])
        self.sld_guard = Slider(ax_guard, 'Guard', 1, 20, valinit=5, valstep=1, orientation='vertical')

        ax_ref = plt.axes([0.05, 0.30, 0.015, 0.60])
        self.sld_ref = Slider(ax_ref, 'Ref', 1, 50, valinit=15, valstep=1, orientation='vertical')

        ax_bias = plt.axes([0.08, 0.30, 0.015, 0.60])
        self.sld_bias = Slider(ax_bias, 'Bias', 0.0, 1.0, valinit=0.1, valstep=0.01, orientation='vertical')

        # RadioButtons para método CFAR
        ax_method = plt.axes([0.01, 0.02, 0.12, 0.20])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        # Todos estos controles disparan la misma actualización
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias):
            ctl.on_changed(lambda v: self.update_display(int(self.slider.val)))
        self.radio_method.on_clicked(lambda label: self.update_display(int(self.slider.val)))

        # Mostrar figura
        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    def update_display(self, idx: int):
        """Dibuja FFT + CFAR solamente en los índices válidos."""
        mag = self.filtered_fft_data[idx, :]
        #mag = self.data[idx, :]

        arr_min = np.min(mag)
        arr_max = np.max(mag)

        if arr_max > arr_min:
            mag = (mag - arr_min) / (arr_max - arr_min)
        else:
            mag = np.zeros_like(mag)

        # Leer valores de los controles
        ng = int(self.sld_guard.val)
        nr = int(self.sld_ref.val)
        b  = float(self.sld_bias.val)
        m  = self.radio_method.value_selected  # etiqueta activa

        # Aplicar CFAR sobre mag filtrado
        thresh, targets = cfar(
            mag,
            num_guard_cells=ng,
            num_ref_cells=nr,
            bias=b,
            cfar_method=m
        )
        det_indices = np.where(targets.mask)[0]

        # Actualizar líneas y puntos
        self.line.set_data(self.freq, mag)
        self.line_thr.set_data(self.freq, thresh)
        self.scatter_det.set_offsets(
            np.c_[self.freq[det_indices], mag[det_indices]]
        )
        # Ajustar límites de X e Y
        #   X: frecuencia válida
        self.ax.set_xlim(self.freq[0], self.freq[-1])
        #   Y: 0 a 1 (normalizado)
        self.ax.set_ylim(0, 1)
        d_min = 0
        d_max = self.freq_to_distance(self.freq[-1])
        self.secax.set_xlim(d_min, d_max)

        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        # 1) Reconstruir matriz original
        arr = np.array(msg.data, dtype=np.float32)
        mat = arr.reshape((msg.rows, msg.cols))
        mat = mat - self.base_data

        # 2) Construir eje de frecuencia completo y corrimiento
        n_frames, n_bins = mat.shape
        raw_freq = np.linspace(-self.sample_rate/2,
                                self.sample_rate/2,
                                n_bins)
        freq_step = raw_freq[1] - raw_freq[0]
        offset_idx = int(self.offset / freq_step)

        beat_freq = raw_freq - self.signal_freq - self.offset
        range_axis = (beat_freq * self.c) / (2 * self.slope)

        # 4) Filtrar solo distancias >= 0
        valid = np.where(range_axis >= 0)[0]
        
        # 5) Desplazar datos en frecuencia y luego filtrar columnas
        rolled = np.roll(mat, -offset_idx, axis=1)
        filtered = rolled[:, valid]
        
        # 6) Guardar para las siguientes llamadas
        self.raw_freq = raw_freq
        self.freq_offset_index = offset_idx
        self.range_axis = range_axis
        self.valid_indices = valid
        self.filtered_fft_data = filtered
        self.freq = raw_freq[valid] - self.offset
        #self.freq = raw_freq - self.offset

        self.data = mat

        # 7) Actualizar slider sin mover thumb
        self.slider.valmax = n_frames - 1
        self.slider.ax.set_xlim(self.slider.valmin, self.slider.valmax)

        # 8) Redibujar en la posición actual del slider
        idx = int(min(self.slider.val, n_frames - 1))
        self.update_display(idx)

        #self.get_logger().info(f'Datos recibidos: {n_frames}x{n_bins}, filtrados a {filtered.shape[1]} bins, mostrando frame {idx}')

    def on_slider_change(self, val: float):
        idx = int(val)
        if self.filtered_fft_data is not None:
            self.update_display(idx)

def main(args=None):
    rclpy.init(args=args)
    node = RadarVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
