#!/usr/bin/env python3
import rclpy # ros2
from rclpy.node import Node # clase nodo de ros2
import numpy as np # calculo matematico
import matplotlib.pyplot as plt # graficos
from radar_msg.msg import RadarData # mensaje de radar
from matplotlib.widgets import Slider, CheckButtons # interfaz de usuario


class RadarRawVisualizer(Node):
    """
    Visualizador de datos CRUDOS de radar.

    A diferencia de radar_polar_visualizer.py, este nodo NO aplica:
      - ventana (win_funct)
      - FFT
      - conversion a dB (s_dbfs)
      - filtro de rango valido (range_filter_min_m / max_m)
      - resta de fondo
      - CFAR

    Solo reconstruye la matriz compleja (data_real + j*data_imag) tal como
    llega en el mensaje RadarData y grafica las muestras en el dominio del
    tiempo (rampa FMCW), en amplitud (I/Q) para el angulo de steering
    seleccionado.
    """

    def __init__(self):
        super().__init__('radar_raw_visualizer')

        # --- parametros configurables ---
        # radar.yaml
        self.declare_parameter('sample_rate_hz', 0.6e6)
        self.declare_parameter('rbeam_angledeg_min', -80)
        self.declare_parameter('rbeam_angledeg_max', 80)
        self.declare_parameter('rbeam_angledeg_step', 1)

        self.sample_rate_hz = self.get_parameter('sample_rate_hz').value
        self.rbeam_angledeg_min = self.get_parameter('rbeam_angledeg_min').value
        self.rbeam_angledeg_max = self.get_parameter('rbeam_angledeg_max').value
        self.rbeam_angledeg_step = self.get_parameter('rbeam_angledeg_step').value

        # suscripcion a datos de radar (sin procesar)
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)

        # matriz cruda completa (n_steering_angle, n_bins) sin ningun filtro
        self.raw_data = None # complejo: data_real + j*data_imag
        self.n_samples = None # eje x = indice de muestra dentro de la rampa
        self.time_axis = None # eje x alternativo en segundos

        # --- figura interactiva ---
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 7))
        plt.subplots_adjust(left=0.12, bottom=0.28, right=0.95, top=0.90)

        # lineas: parte real, parte imaginaria, magnitud (sin dB, sin normalizar)
        self.line_real, = self.ax.plot([], [], lw=1.5, label='I (Real)')
        self.line_imag, = self.ax.plot([], [], lw=1.5, label='Q (Imag)')
        self.line_mag, = self.ax.plot([], [], lw=1.2, ls='--', label='|Magnitud|')
        self.line_mag.set_visible(False)

        self.ax.set_xlabel("Muestra (n)")
        self.ax.set_ylabel("Amplitud cruda (ADC counts)")
        self.ax.set_title("Datos crudos de radar (sin ventana / sin FFT / sin dB)")

        # eje secundario en tiempo [s], util para verificar la rampa FMCW
        self.secax = self.ax.secondary_xaxis(
            'top',
            functions=(self._sample_to_time, self._time_to_sample)
        )
        self.secax.set_xlabel("Tiempo [s]")

        # slider de angulo de steering (fila de la matriz)
        ax_slider = plt.axes([0.20, 0.10, 0.65, 0.03])
        init_angle = np.clip(0, self.rbeam_angledeg_min, self.rbeam_angledeg_max) \
            if self.rbeam_angledeg_min <= 0 <= self.rbeam_angledeg_max else self.rbeam_angledeg_min
        self.sld_angle = Slider(
            ax_slider, 'Steering angle',
            self.rbeam_angledeg_min, self.rbeam_angledeg_max,
            valinit=init_angle, valstep=self.rbeam_angledeg_step
        )
        self.sld_angle.on_changed(self.on_slider_change)

        # checkboxes para elegir que trazas mostrar (I, Q, magnitud)
        ax_check = plt.axes([0.01, 0.75, 0.10, 0.15])
        self.check_traces = CheckButtons(ax_check, ['I (Real)', 'Q (Imag)', '|Mag|'], [True, True, False])
        self.check_traces.on_clicked(self.on_check_change)

        self.ax.legend(loc='upper right')

        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    # --- helpers de conversion de eje ---
    def _sample_to_time(self, n):
        return n / self.sample_rate_hz

    def _time_to_sample(self, t):
        return t * self.sample_rate_hz

    def on_check_change(self, label):
        status = self.check_traces.get_status()
        self.line_real.set_visible(status[0])
        self.line_imag.set_visible(status[1])
        self.line_mag.set_visible(status[2])
        self.fig.canvas.draw_idle()

    def on_slider_change(self, val: float):
        angle_val = int(val)
        if self.raw_data is not None:
            self.update_display(angle_val)

    def ang_to_idx(self, ang: int) -> int:
        """Convierte un angulo (deg) a indice de fila [0..rows-1]."""
        if self.raw_data is None:
            return 0
        rows = self.raw_data.shape[0]
        idx = int(round((ang - self.rbeam_angledeg_min) / float(self.rbeam_angledeg_step)))
        return max(0, min(rows - 1, idx))

    def update_display(self, angle_val: int):
        """Dibuja las muestras crudas (I, Q, |Mag|) de la fila seleccionada."""
        if self.raw_data is None:
            return

        idx = self.ang_to_idx(angle_val)
        row = self.raw_data[idx, :] # vector complejo crudo, sin procesar

        n = np.arange(self.n_samples)
        real_part = row.real
        imag_part = row.imag
        mag_part = np.abs(row) # magnitud cruda, SIN escala dB

        self.line_real.set_data(n, real_part)
        self.line_imag.set_data(n, imag_part)
        self.line_mag.set_data(n, mag_part)

        self.ax.set_xlim(n[0], n[-1])
        self.secax.set_xlim(n[0], n[-1])

        # limites Y automaticos segun datos crudos visibles
        visible_vals = []
        if self.line_real.get_visible():
            visible_vals.append(real_part)
        if self.line_imag.get_visible():
            visible_vals.append(imag_part)
        if self.line_mag.get_visible():
            visible_vals.append(mag_part)

        if visible_vals:
            all_vals = np.concatenate(visible_vals)
            margin = 0.1 * (np.max(all_vals) - np.min(all_vals) + 1e-9)
            self.ax.set_ylim(np.min(all_vals) - margin, np.max(all_vals) + margin)

        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')

        try:
            np_dtype = np.dtype(msg.dtype)
        except Exception:
            np_dtype = np.float64

        rows, cols = [msg.rows, msg.cols]

        # reconstruccion de la matriz cruda, sin ningun procesamiento posterior
        data_real = np.asarray(msg.data_real, dtype=np_dtype).reshape((rows, cols))
        data_imag = np.asarray(msg.data_imag, dtype=np_dtype).reshape((rows, cols))
        self.raw_data = data_real + 1j * data_imag # SIN ventana, SIN FFT, SIN dB

        self.n_samples = cols

        # mostrar/ocultar slider de angulo segun cantidad de filas
        self.sld_angle.ax.set_visible(rows > 1)

        angle_val = self.sld_angle.val
        self.update_display(angle_val)


def main(args=None):
    rclpy.init(args=args)
    node = RadarRawVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Nodo interrumpido por el usuario.")
    except Exception as e:
        print(f"Excepcion no controlada: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()