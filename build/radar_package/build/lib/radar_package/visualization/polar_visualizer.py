#!/usr/bin/env python3
import rclpy # ros2
from rclpy.node import Node # clase nodo de ros2
import numpy as np # calculo matematico
import matplotlib.pyplot as plt # graficos
from radar_msg.msg import RadarData # mensaje de radar
from matplotlib.widgets import Slider, RadioButtons

import os
#from ament_index_python.packages import get_package_share_directory # recursos

class RadarPolarVisualizer(Node):
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

        # suscripción a datos de radar
        # los datos son recibidos como una matriz fft de frecuencias en steering angle
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)

        # Ejes y datos que se rellenan en el primer mensaje
        self.filtered_amplitude = None # amplitud filtrada "y" desplazada en offset
        self.filtered_phase = None # fase filtada "y" desplazada en offset
        self.filtered_freq = None # eje "x" filtrado
        self.freq = None # eje de frecuencias
        self.valid_indices = None # índices >= 0 m

        # aplicacion de ventana a los datos antes de la FFT
        self.win_funct = np.ones(self.good_ramp_samples, dtype=np.float64) # ventana rectangular
        #self.win_funct = np.blackman(self.good_ramp_samples) # ventana blackman -> posiblemente se deba considerar offset
        #self.win_funct = np.hamming(self.good_ramp_samples)
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

        # informacion de los ejes
        self.ax.set_xlabel("Range [m]")
        self.ax.set_ylabel("Magnitude (dB)") # normalizada min-max

        # eje secundario de frecuencia en la parte superior
        self.secax = self.ax.secondary_xaxis('top', functions=(self.distance_to_freq, self.freq_to_distance))
        self.secax.set_xlabel("Frequency [Hz]")

        # Slider de frames (recorrido en steering angle)
        ax_slider = plt.axes([0.25, 0.05, 0.65, 0.03])
        init_angle = np.clip(0, self.rbeam_angledeg_min, self.rbeam_angledeg_max) if self.rbeam_angledeg_min <= 0 <= self.rbeam_angledeg_max else self.rbeam_angledeg_min
        self.sld_angle = Slider(ax_slider, 'Steering angle', self.rbeam_angledeg_min, self.rbeam_angledeg_max, valinit=init_angle, valstep=self.rbeam_angledeg_step)

        # CONTROLES INTERACTIVOS
        # Slider para cfar_guard_cells
        ax_guard = plt.axes([0.015, 0.30, 0.015, 0.60])
        self.sld_guard = Slider(ax_guard, "Guard\n(N)", 1, 30, valinit=self.cfar_guard_cells, valstep=1, orientation='vertical')

        # Slider para cfar_reference_cells
        ax_ref = plt.axes([0.045, 0.30, 0.015, 0.60])
        self.sld_ref = Slider(ax_ref, "Ref\n(N)", 1, 70, valinit=self.cfar_reference_cells, valstep=1, orientation='vertical')

        # Slider para cfar_bias
        ax_bias = plt.axes([0.075, 0.30, 0.015, 0.60])
        self.sld_bias = Slider(ax_bias, "Bias\n(dB)", 0.0, 30.0, valinit=self.cfar_bias, valstep=1, orientation='vertical')

        # Slider para fa_rate (solo para método false_alarm)
        ax_fa = plt.axes([0.105, 0.30, 0.015, 0.60]) 
        self.sld_fa = Slider(ax_fa, 'FA\nRate', 0.0, 2.0, valinit=0.5, valstep=0.01, orientation='vertical')
        self.sld_fa.ax.set_visible(False)

        # RadioButtons para método CFAR
        ax_method = plt.axes([0.01, 0.02, 0.12, 0.15])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        self.sld_angle.on_changed(self.on_slider_change) # callback ante eventos
        # disparo de actualización
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias, self.sld_fa):
            ctl.on_changed(lambda v: self.update_display(int(self.sld_angle.val)))
        self.radio_method.on_clicked(lambda label: self.update_display(int(self.sld_angle.val)))

        # mostrar figura
        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    



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
