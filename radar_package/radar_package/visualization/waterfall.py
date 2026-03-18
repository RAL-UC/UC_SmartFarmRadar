#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from radar_msg.msg import RadarData
#from radar_package.parametros import *  # C, SLOPE, SIGNAL_FREQ, OFFSET, ANGLE_MIN, ANGLE_MAX, ANGLE_STEP, SAMPLE_RATE

# Nodo ROS
class RadarWaterfall(Node):
    def __init__(self):
        super().__init__('radar_waterfall')

        # parámetros
        # radar.yaml
        self.declare_parameter('sample_rate_hz', 0.6e6)
        self.declare_parameter('signal_freq_hz', 100e3)
        self.declare_parameter('range_offset_hz', 10760.0)
        self.declare_parameter('speed_of_light', 3e8)

        # carga de valores
        self.sample_rate_hz = self.get_parameter('sample_rate_hz').value
        self.signal_freq_hz = self.get_parameter('signal_freq_hz').value
        self.range_offset_hz = self.get_parameter('range_offset_hz').value
        self.speed_of_light = self.get_parameter('speed_of_light').value



        self.declare_parameter('topic', 'radar_data')
        self.declare_parameter('num_slices', 100)           # alto del waterfall (n° de filas/tiempo)
        self.declare_parameter('db_low', -60.0)             # nivel bajo (dB) para clim
        self.declare_parameter('db_high', 0.0)              # nivel alto (dB) para clim
        self.declare_parameter('angle_min', ANGLE_MIN)
        self.declare_parameter('angle_max', ANGLE_MAX)
        self.declare_parameter('angle_step', ANGLE_STEP)
        # Ruta a medición de fondo (.npy). Por defecto, intenta resource/medicion_fondo.npy
        try:
            pkg_share = get_package_share_directory('radar_package')
            default_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo.npy')
        except Exception:
            default_fondo = ''
        self.declare_parameter('path_medicion_fondo', default_fondo)

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.num_slices = int(self.get_parameter('num_slices').get_parameter_value().integer_value)
        self.db_low = float(self.get_parameter('db_low').get_parameter_value().double_value)
        self.db_high = float(self.get_parameter('db_high').get_parameter_value().double_value)
        self.angle_min = int(self.get_parameter('angle_min').get_parameter_value().integer_value)
        self.angle_max = int(self.get_parameter('angle_max').get_parameter_value().integer_value)
        self.angle_step = int(self.get_parameter('angle_step').get_parameter_value().integer_value)
        self.path_fondo = self.get_parameter('path_medicion_fondo').get_parameter_value().string_value

        # -------- Cargar medición de fondo ----------
        self.medicion_fondo = None
        if self.path_fondo and os.path.exists(self.path_fondo):
            try:
                self.medicion_fondo = np.load(self.path_fondo)
                self.get_logger().info(f"medicion_fondo cargado: {self.path_fondo} shape={self.medicion_fondo.shape}")
            except Exception as e:
                self.get_logger().warn(f"No se pudo cargar medicion_fondo: {e}")

        # -------- Subscripción ----------
        self.create_subscription(RadarData, self.topic, self.listener_callback, 10)

        # -------- Estado de datos ----------
        self.freq = None                # eje de frecuencia válido (>=0 m)
        self.x_range = None             # eje rango [m] de los bins válidos
        self.filtered_data = None       # [n_rows, n_bins_valid]
        self.img_array = None           # waterfall buffer [num_slices, n_bins_valid] en dB
        self.current_idx = 0            # índice de fila (según slider)
        self._lock = threading.Lock()   # proteger acceso desde callback ROS

        # ============ GUI (Matplotlib en hilo principal) ============
        self.fig = plt.figure(figsize=(14, 8), constrained_layout=False)

        # Rectángulos [left, bottom, width, height] en coords de figura
        RECT_ANGLE = [0.08, 0.94, 0.84, 0.035]  # slider superior (fino)
        RECT_LINE = [0.08, 0.60, 0.84, 0.28]   # curva 1D
        RECT_IMG = [0.08, 0.26, 0.84, 0.28]   # waterfall
        RECT_CBAR = [0.93, 0.26, 0.02, 0.28]   # colorbar del waterfall (opcional)
        RECT_WL_LOW = [0.08, 0.09, 0.36, 0.035]  # slider WL Low (dB)
        RECT_WL_HIGH = [0.56, 0.09, 0.36, 0.035]  # slider WL High (dB)

        # --- Curva 1D (arriba) ---
        self.ax_line = self.fig.add_axes(RECT_LINE)
        (self.line_spec,) = self.ax_line.plot([], [], lw=1.6, label='Magnitude [dB]')
        self.ax_line.set_xlabel("Range [m]", labelpad=8)
        self.ax_line.set_ylabel("dB", labelpad=8)
        self.ax_line.grid(True, alpha=0.25)
        self.ax_line.tick_params(labelsize=9)

        # Eje secundario superior: frecuencia
        self.secax = self.ax_line.secondary_xaxis('top', functions=(distance_to_freq, freq_to_distance))
        self.secax.set_xlabel("Frequency [Hz]")
        self.secax.tick_params(labelsize=9, pad=8)

        # --- Waterfall (al medio) ---
        self.ax_img = self.fig.add_axes(RECT_IMG)
        self.ax_img.set_xlabel("Range [m]", labelpad=12)      # +pad para despegar de sliders de abajo
        self.ax_img.set_ylabel("Time (slices)", labelpad=8)
        self.ax_img.tick_params(labelsize=9)
        self.im = None
        # Colorbar opcional a la derecha
        self.ax_cbar = self.fig.add_axes(RECT_CBAR)
        self.cbar = None

        # --- Slider de ángulo (arriba de todo) ---
        angle_ax = self.fig.add_axes(RECT_ANGLE)
        init_angle = float(np.clip(0.0, self.angle_min, self.angle_max))
        self.sld_angle = Slider(angle_ax, 'Steering angle',
                                self.angle_min, self.angle_max,
                                valinit=init_angle, valstep=self.angle_step)

        # --- Sliders WL Low / WL High (abajo, separados) ---
        low_ax  = self.fig.add_axes(RECT_WL_LOW)
        high_ax = self.fig.add_axes(RECT_WL_HIGH)
        self.sld_low  = Slider(low_ax,  'WL Low (dB)',  -80.0, 40.0,  valinit=self.db_low)
        self.sld_high = Slider(high_ax, 'WL High (dB)', -40.0, 80.0, valinit=self.db_high)

        # Mueve el número de cada slider fuera de la barra para que no se pisen
        for sld in (self.sld_low, self.sld_high):
            sld.label.set_fontsize(9)
            sld.valtext.set_fontsize(9)
            sld.valtext.set_transform(sld.ax.transAxes)
            sld.valtext.set_position((1.02, 0.5))  # a la derecha de la pista

        # Handlers con pequeño debounce
        self._mpl_timer = None
        self._pending_draw = False
        self.sld_angle.on_changed(self._on_angle_change)
        self.sld_low.on_changed(lambda _v: self._schedule_redraw())
        self.sld_high.on_changed(lambda _v: self._schedule_redraw())

        self.fig.canvas.draw_idle()
        # ===========================================================

    # ------------- Handlers GUI -------------
    def _on_angle_change(self, val):
        with self._lock:
            if self.filtered_data is None:
                self.current_idx = 0
                return
            self.current_idx = self.angle_to_index(float(val), self.angle_min, self.angle_max, self.filtered_data.shape[0])
        self._schedule_redraw()

    def _schedule_redraw(self, delay_ms=20):
        self._pending_draw = True
        if self._mpl_timer is not None:
            try:
                self._mpl_timer.stop()
            except Exception:
                pass
        self._mpl_timer = self.fig.canvas.new_timer(interval=delay_ms)
        self._mpl_timer.single_shot = True
        self._mpl_timer.add_callback(self._redraw_mainthread)
        self._mpl_timer.start()

    def _redraw_mainthread(self):
        if not self._pending_draw:
            return
        self._pending_draw = False
        with self._lock:
            if self.filtered_data is None or self.freq is None or self.x_range is None:
                return
            idx = int(np.clip(self.current_idx, 0, self.filtered_data.shape[0]-1))
            mag = self.filtered_data[idx, :]

            # Magnitud en dB relativa al pico (evita log(0))
            #eps = 1e-15
            #mag_norm = mag / max(np.max(np.abs(mag)), eps)
            #s_db = 20.0 * np.log10(np.maximum(np.abs(mag_norm), eps))

            # Actualiza traza 1D
            self.line_spec.set_data(self.x_range, mag)
            self.ax_line.set_xlim(self.x_range[0], self.x_range[-1])
            y_lo, y_hi = float(np.min(mag)), float(np.max(mag))
            if y_lo == y_hi:
                y_lo, y_hi = y_lo - 1.0, y_hi + 1.0
            self.ax_line.set_ylim(y_lo, y_hi)
            self.secax.set_xlim(self.freq[0], self.freq[-1])

            # Inserta la fila más reciente en el waterfall (fila 0 = más reciente)
            if self.img_array is not None:
                self.img_array = np.roll(self.img_array, 1, axis=0)
                # escribe la captura más reciente en la fila 0 (abajo):
                self.img_array[0, :] = mag

                if self.im is None:
                    # con origin='lower' la fila 0 queda abajo
                    extent = [self.x_range[0], self.x_range[-1], 0, self.num_slices]
                    self.im = self.ax_img.imshow(
                        self.img_array, aspect='auto', origin='lower', extent=extent, cmap='viridis'
                    )
                    # (si usas colorbar)
                    if hasattr(self, 'ax_cbar'):
                        self.cbar = self.fig.colorbar(self.im, cax=self.ax_cbar)
                        self.cbar.set_label("dB", rotation=270, labelpad=10)
                else:
                    self.im.set_data(self.img_array)
                    self.im.set_extent([self.x_range[0], self.x_range[-1], 0, self.num_slices])
                    if getattr(self, 'cbar', None) is not None:
                        self.cbar.update_normal(self.im)

                # niveles (clim)
                lo = min(self.sld_low.val, self.sld_high.val - 1e-3)
                hi = self.sld_high.val
                self.im.set_clim(vmin=lo, vmax=hi)

        self.fig.canvas.draw_idle()

    # ------------- Callback ROS -------------
    def listener_callback(self, msg: RadarData):
        # Reconstruye matriz (rows x cols)
        try:
            arr = np.array(msg.data, dtype=msg.dtype)
            mat = arr.reshape((msg.rows, msg.cols))
        except Exception as e:
            self.get_logger().error(f"Error reshape RadarData: {e}")
            return

        # Resta medición de fondo si coincide la forma
        if self.medicion_fondo is not None and self.medicion_fondo.shape == mat.shape:
            #mat = mat - self.medicion_fondo
            mat = mat
        elif self.medicion_fondo is not None and self.medicion_fondo.shape != mat.shape:
            self.get_logger().warn(f"Shape fondo {self.medicion_fondo.shape} != datos {mat.shape}; omitiendo resta.")

        # Eje de frecuencias y filtro por distancia >= 0
        n_bins = msg.cols
        freq_full = np.linspace(-self.sample_rate_hz/2.0, self.sample_rate_hz/2.0, n_bins, endpoint=False)
        dist_full = self.freq_to_distance(freq_full)
        valid = np.where(dist_full >= 0.0)[0]
        if valid.size == 0:
            self.get_logger().warn("No hay bins válidos (dist >= 0).")
            return

        with self._lock:
            self.freq = freq_full[valid]
            self.x_range = dist_full[valid]
            self.filtered_data = mat[:, valid]

            # Inicializa buffer del waterfall si aún no existe o cambió ancho
            width = self.filtered_data.shape[1]
            if self.img_array is None or self.img_array.shape[1] != width:
                self.img_array = np.ones((self.num_slices, width), dtype=np.float64) * (-120.0) # float32

            # Calcula índice según slider actual y agenda redibujo
            angle = float(self.sld_angle.val)
            self.current_idx = self.angle_to_index(angle, self.angle_min, self.angle_max, self.filtered_data.shape[0])

        self._schedule_redraw()

    # Utilidades ejes y mapeos
    def freq_to_distance(self, f):
        # misma convención que has usado en tus nodos
        return (f - self.signal_freq_hz - self.range_offset_hz) * C / (2.0 * SLOPE)

    def distance_to_freq(self, d):
        return self.signal_freq_hz + self.range_offset_hz + (d * 2.0 * SLOPE / C)

    def angle_to_index(self, angle, amin, amax, n_rows):
        if n_rows <= 1 or amax == amin:
            return 0
        alpha = (angle - amin) / (amax - amin)
        return int(np.clip(round(alpha * (n_rows - 1)), 0, n_rows - 1))

# ===========================
# main: ROS en hilo aparte, GUI en principal
# ===========================
def main(args=None):
    rclpy.init(args=args)
    node = RadarWaterfall()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        plt.show()  # GUI loop en el hilo principal
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
