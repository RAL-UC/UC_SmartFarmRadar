#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, RadioButtons, CheckButtons
from radar_msg.msg import RadarData
from radar_package.processing.target_detection_dbfs import cfar  # objetivos de deteccion
import os
from ament_index_python.packages import get_package_share_directory  # recursos

# recursos
pkg_share = get_package_share_directory('radar_package')
# filtro resta de fondo
path_medicion_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo_centro2.npy')


class RadarPolarVisualizer(Node):
    """
    Visualizador tipo PPI (Plan Position Indicator): representa en coordenadas
    polares la magnitud (dB) de todo el barrido de steering angles vs. distancia,
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
        self.subscription = self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)

        self.medicion_fondo = np.load(path_medicion_fondo)  # carga medicion de fondo en datos de radar

        # Funciones de conversión freq <-> range
        self.freq_to_distance = lambda f: (f - self.signal_freq_hz - self.range_offset_hz) * self.speed_of_light / (2 * self.slope_hz_per_s)
        self.distance_to_freq = lambda d: self.signal_freq_hz + self.range_offset_hz + (d * 2 * self.slope_hz_per_s / self.speed_of_light)

        # Datos que se rellenan en el primer mensaje
        self.filtered_data = None  # matriz (angulos x rango) filtrada en dB
        self.freq = None           # eje de frecuencias filtrado
        self.theta = None          # steering angles en radianes (uno por fila de filtered_data)
        self.valid_indices = None  # índices de distancia válidos

        # ventana aplicada antes de la FFT
        self.win_funct = np.ones(self.good_ramp_samples, dtype=np.float64)
        self.sum_win_funct = np.sum(self.win_funct)

        # Configuración de Matplotlib interactivo
        plt.ion()
        self.fig = plt.figure(figsize=(10, 9))
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_theta_zero_location('N')   # 0° apuntando "hacia adelante"
        self.ax.set_theta_direction(-1)        # ángulos positivos hacia la derecha
        self.ax.set_thetamin(self.rbeam_angledeg_min)
        self.ax.set_thetamax(self.rbeam_angledeg_max)
        self.ax.set_title("PPI - Barrido de radar (steering angle vs. distancia)", pad=20)
        self.ax.set_rlabel_position(135)

        self.mesh = None       # QuadMesh de magnitud dB
        self.cbar = None       # colorbar de magnitud
        self.scatter_det = self.ax.scatter([], [], s=25, c='red', marker='x',
                                            label='Detecciones CFAR', zorder=5)

        # Reservar espacio inferior (CFAR) e izquierdo (escala de color) para controles
        plt.subplots_adjust(left=0.18, bottom=0.32, right=0.90, top=0.90)

        # CONTROLES INTERACTIVOS (escala de color, vmin/vmax)
        # rango amplio para cubrir distintas configuraciones de ganancia / resta de fondo
        ax_vmin = plt.axes([0.03, 0.32, 0.015, 0.58])
        self.sld_vmin = Slider(ax_vmin, "vmin\n(dB)", -150.0, 50.0, valinit=-80.0, valstep=1.0,
                                orientation='vertical')

        ax_vmax = plt.axes([0.08, 0.32, 0.015, 0.58])
        self.sld_vmax = Slider(ax_vmax, "vmax\n(dB)", -150.0, 150.0, valinit=50.0, valstep=1.0,
                                orientation='vertical')

        # CONTROLES INTERACTIVOS (CFAR)
        ax_guard = plt.axes([0.10, 0.20, 0.32, 0.02])
        self.sld_guard = Slider(ax_guard, "Guard (N)", 1, 30, valinit=self.cfar_guard_cells, valstep=1)

        ax_ref = plt.axes([0.10, 0.16, 0.32, 0.02])
        self.sld_ref = Slider(ax_ref, "Ref (N)", 1, 70, valinit=self.cfar_reference_cells, valstep=1)

        ax_bias = plt.axes([0.10, 0.12, 0.32, 0.02])
        self.sld_bias = Slider(ax_bias, "Bias (dB)", 0.0, 30.0, valinit=self.cfar_bias, valstep=1)

        ax_fa = plt.axes([0.10, 0.08, 0.32, 0.02])
        self.sld_fa = Slider(ax_fa, "FA Rate", 0.0, 2.0, valinit=0.5, valstep=0.01)
        self.sld_fa.ax.set_visible(False)

        ax_method = plt.axes([0.52, 0.04, 0.16, 0.20])
        self.radio_method = RadioButtons(ax_method, ['average', 'greatest', 'smallest', 'false_alarm'], active=0)

        ax_check = plt.axes([0.72, 0.10, 0.20, 0.10])
        self.chk_det = CheckButtons(ax_check, ['Mostrar detecciones'], [True])

        # callbacks
        for ctl in (self.sld_guard, self.sld_ref, self.sld_bias, self.sld_fa,
                    self.sld_vmin, self.sld_vmax):
            ctl.on_changed(lambda v: self.update_display())
        self.radio_method.on_clicked(lambda label: self.update_display())
        self.chk_det.on_clicked(lambda label: self.update_display())

        plt.show(block=False)
        self.create_timer(0.05, lambda: plt.pause(0.001))

    def _set_fa_visible(self, visible: bool):
        # mostrar/ocultar slider de fa_rate según método CFAR seleccionado
        self.sld_fa.ax.set_visible(bool(visible))
        self.fig.canvas.draw_idle()

    def extend_with_means(self, mag, total_guard_ref):
        """Extiende el vector con el promedio de sus extremos, para evitar bordes en el CFAR."""
        mean_start = np.mean(mag[:total_guard_ref])
        mean_end = np.mean(mag[-total_guard_ref:])
        pad_start = np.full(total_guard_ref, mean_start)
        pad_end = np.full(total_guard_ref, mean_end)
        return np.concatenate([pad_start, mag, pad_end])

    def unpad(self, v, total_guard_ref):
        return v[total_guard_ref:-total_guard_ref]

    def compute_cfar_mask(self, mat):
        """Aplica CFAR fila por fila (un ángulo de steering a la vez) y devuelve
        una máscara booleana (angulos x rango) con las detecciones."""
        ng = int(self.sld_guard.val)
        nr = int(self.sld_ref.val)
        b = float(self.sld_bias.val)
        m = self.radio_method.value_selected
        fa_rate = float(self.sld_fa.val)
        total_ext = ng + nr

        rows, cols = mat.shape
        det_mask = np.zeros((rows, cols), dtype=bool)

        use_fa = (m == "false_alarm")
        self._set_fa_visible(use_fa)

        for i in range(rows):
            mag_ext = self.extend_with_means(mat[i, :], total_ext)
            if use_fa:
                _, targets, _ = cfar(mag_ext, num_guard_cells=ng, num_ref_cells=nr,
                                      bias=b, cfar_method=m, fa_rate=fa_rate)
            else:
                _, targets = cfar(mag_ext, num_guard_cells=ng, num_ref_cells=nr,
                                   bias=b, cfar_method=m)
            targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
            det_mask[i, :] = targets.mask

        return det_mask

    def update_display(self):
        """Redibuja el diagrama polar completo (magnitud + detecciones CFAR)."""
        if self.filtered_data is None or self.freq is None or self.filtered_data.size == 0:
            return

        mat = self.filtered_data                    # (angulos x rango), en dB
        r = self.freq_to_distance(self.freq)         # eje de distancia
        theta = self.theta                           # eje angular, en radianes

        THETA, R = np.meshgrid(theta, r, indexing='ij')

        # límites de la escala de color, tomados de los sliders vmin/vmax
        vmin = float(self.sld_vmin.val)
        vmax = float(self.sld_vmax.val)
        if vmin >= vmax:
            # evita un rango inválido mientras el usuario ajusta las barras
            vmax = vmin + 1.0

        if self.mesh is not None:
            self.mesh.remove()
        self.mesh = self.ax.pcolormesh(THETA, R, mat, shading='auto', cmap='viridis',
                                        vmin=vmin, vmax=vmax)
        if self.cbar is None:
            self.cbar = self.fig.colorbar(self.mesh, ax=self.ax, pad=0.12, shrink=0.8)
            self.cbar.set_label("Magnitud (dB)")
        else:
            self.cbar.update_normal(self.mesh)

        show_det = self.chk_det.get_status()[0]
        if show_det:
            det_mask = self.compute_cfar_mask(mat)
            det_rows, det_cols = np.where(det_mask)
            theta_det = theta[det_rows]
            r_det = r[det_cols]
            self.scatter_det.set_offsets(np.c_[theta_det, r_det])
            self.scatter_det.set_visible(True)
        else:
            self.scatter_det.set_visible(False)

        self.ax.set_rmax(r[-1] if r[-1] > 0 else 1.0)
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.25, 1.1))
        self.fig.canvas.draw_idle()

    def listener_callback(self, msg: RadarData):
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')
        try:
            np_dtype = np.dtype(msg.dtype)
        except Exception:
            np_dtype = np.float64
        rows, cols = [msg.rows, msg.cols]

        data_real = np.asarray(msg.data_real, dtype=np_dtype).reshape((rows, cols))
        data_imag = np.asarray(msg.data_imag, dtype=np_dtype).reshape((rows, cols))
        mat = data_real + 1j * data_imag

        mat[:, :self.good_ramp_samples] *= self.win_funct[None, :]
        sp = np.fft.fftshift(np.fft.fft(mat, axis=1), axes=1)
        s_mag = np.abs(sp) / self.sum_win_funct
        s_mag = np.maximum(s_mag, 10 ** (-15))
        mat = 20 * np.log10(s_mag / (2 ** 11))  # s_dbfs

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
        freq = np.linspace(-self.sample_rate_hz / 2, self.sample_rate_hz / 2, cols, endpoint=False)
        distance = self.freq_to_distance(freq)
        self.valid_indices = np.where((distance >= self.range_filter_min_m) & (distance <= self.range_filter_max_m))[0]
        self.filtered_data = mat[:, self.valid_indices]
        self.freq = freq[self.valid_indices]

        # eje angular: un steering angle por fila de la matriz
        angles_deg = self.rbeam_angledeg_min + np.arange(rows) * self.rbeam_angledeg_step
        self.theta = np.deg2rad(angles_deg)

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