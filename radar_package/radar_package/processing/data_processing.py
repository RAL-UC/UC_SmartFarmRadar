#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header

from radar_msg.msg import RadarData
from radar_msg.msg import Ptu
from radar_msg.msg import RadarCartesian

from radar_package.processing.target_detection_dbfs import cfar
#from radar_package.parametros import *

import os
from ament_index_python.packages import get_package_share_directory

class RadarSphericalToCartesian(Node):
    """
    De /radar_data realiza detecciones y la transformación de coordenadas esfericas a cartesianas (x,y,z),
    utiliza los datos de pan/tilt del PTU y la rejilla de ángulos de beamforming.
    Publica puntos en /radar_cartesian.
    theta_pan
    phi_tilt
    theta_beam
    """

    def __init__(self):
        super().__init__('radar_spherical_to_cartesian')

        # parametros
        # radar.yaml
        self.declare_parameter('sample_rate_hz', 0.6e6)
        self.declare_parameter('signal_freq_hz', 100e3)
        self.declare_parameter('good_ramp_samples', 270)
        self.declare_parameter('range_offset_hz', 10760.0)
        self.declare_parameter('slope_hz_per_s', 1.0e12)
        self.declare_parameter('speed_of_light', 3e8)
        # barrido en azimuth theta_beam
        self.declare_parameter('rbeam_angledeg_min', -80)
        self.declare_parameter('rbeam_angledeg_max', 80)
        self.declare_parameter('rbeam_angledeg_step', 1)
        # processing.yaml
        # parámetros CFAR
        self.declare_parameter('cfar_guard_cells', 15)
        self.declare_parameter('cfar_reference_cells', 45)
        self.declare_parameter('cfar_bias', 10)
        self.declare_parameter('cfar_method', 'average')
        # filtro de distancias
        self.declare_parameter('range_filter_min_m', 0.0)
        self.declare_parameter('range_filter_max_m', 5.0)

        # carga de valores
        #radar.yaml
        self.sample_rate_hz = self.get_parameter('sample_rate_hz').value
        self.signal_freq_hz = self.get_parameter('signal_freq_hz').value
        self.good_ramp_samples = self.get_parameter('good_ramp_samples').value
        self.range_offset_hz = self.get_parameter('range_offset_hz').value
        self.slope_hz_per_s = self.get_parameter('slope_hz_per_s').value
        self.speed_of_light = self.get_parameter('speed_of_light').value
        self.rbeam_angledeg_min = self.get_parameter('rbeam_angledeg_min').value
        self.rbeam_angledeg_max = self.get_parameter('rbeam_angledeg_max').value
        self.rbeam_angledeg_step = self.get_parameter('rbeam_angledeg_step').value
        # processing.yaml
        self.cfar_guard_cells = self.get_parameter('cfar_guard_cells').value
        self.cfar_reference_cells = self.get_parameter('cfar_reference_cells').value
        self.cfar_bias = self.get_parameter('cfar_bias').value
        self.cfar_method = self.get_parameter('cfar_method').value
        self.range_filter_min_m = self.get_parameter('range_filter_min_m').value
        self.range_filter_max_m = self.get_parameter('range_filter_max_m').value

        # suscriptor y publicador
        self.sub_radar = self.create_subscription(RadarData, 'radar_data', self.cb_radar, 10)
        self.sub_ptu = self.create_subscription(Ptu, 'ptu_data', self.cb_ptu, 10)
        self.pub = self.create_publisher(RadarCartesian, 'radar_cartesian', 10)

        self.last_pan_deg = None
        self.last_tilt_deg = None

        # Recursos: medición de fondo
        try:
            pkg_share = get_package_share_directory('radar_package')
            self.path_medicion_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo_centro2.npy')
            self.medicion_fondo = np.load(self.path_medicion_fondo) # shape esperada (rows, cols) o (1, cols)
            self.get_logger().info(f"Cargada medición de fondo desde: {self.path_medicion_fondo}")
        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar medición de fondo: {e!r}. Usando ceros.")
            self.medicion_fondo = None
        
        self.win_funct = np.ones(self.good_ramp_samples, dtype=np.float64) # ventana rectangular
        #self.win_funct = np.blackman(self.good_ramp_samples) # ventana blackman -> posiblemente se deba considerar offset
        #self.win_funct = np.hamming(self.good_ramp_samples)
        self.sum_win_funct = np.sum(self.win_funct)

        # conversión frecuencia <-> distancia
        self.freq_to_distance = lambda f: (f - self.signal_freq_hz - self.range_offset_hz) * self.speed_of_light / (2.0 * self.slope_hz_per_s)
        self.distance_to_freq = lambda d: self.signal_freq_hz + self.range_offset_hz + (d * 2.0 * self.slope_hz_per_s / self.speed_of_light)  

        # construir vector de theta_beam
        self.theta_beam_deg = np.arange(self.rbeam_angledeg_min, self.rbeam_angledeg_max + 1, self.rbeam_angledeg_step)
        #self.get_logger().info(f"theta_beam_deg: {self.theta_beam_deg}")

        self.get_logger().info("Nodo listo: radar_spherical_to_cartesian a /radar_cartesian")

    # ------------------ Utilidades CFAR ------------------

    def extend_with_means(self, mag: np.ndarray, total_guard_ref: int) -> np.ndarray:
        mean_start = float(np.mean(mag[:total_guard_ref])) if total_guard_ref > 0 else float(np.mean(mag))
        mean_end   = float(np.mean(mag[-total_guard_ref:])) if total_guard_ref > 0 else float(np.mean(mag))
        pad_start  = np.full(total_guard_ref, mean_start, dtype=mag.dtype)
        pad_end    = np.full(total_guard_ref, mean_end,   dtype=mag.dtype)
        return np.concatenate([pad_start, mag, pad_end])

    def unpad(self, v: np.ndarray, total_guard_ref: int) -> np.ndarray:
        return v[total_guard_ref:-total_guard_ref] if total_guard_ref > 0 else v
    
    # PTU
    def cb_ptu(self, msg: Ptu):
        self.last_pan_deg = msg.pan_deg
        self.last_tilt_deg = msg.tilt_deg

    # ------------------ Callback principal ------------------

    def cb_radar(self, msg: RadarData):
        if self.last_pan_deg is None or self.last_tilt_deg is None:
            self.get_logger().warn("Aún no hay datos de PTU")
            return
        
        pan_deg  = self.last_pan_deg
        tilt_deg = self.last_tilt_deg
        
        # 1) Reconstruir matriz (n_steer × n_bins)
        try:
            np_dtype = np.dtype(msg.dtype)
        except Exception:
            np_dtype = np.float64

        rows, cols = int(msg.rows), int(msg.cols)
        
        data_real = np.asarray(msg.data_real, dtype=np_dtype).reshape((rows, cols))
        data_imag = np.asarray(msg.data_imag, dtype=np_dtype).reshape((rows, cols))
        mat = data_real + 1j * data_imag

        mat[:,:self.good_ramp_samples] *=  self.win_funct[None, :]
        sp = np.fft.fftshift(np.fft.fft(mat, axis=1), axes=1)
        s_mag = np.abs(sp) / self.sum_win_funct
        s_mag = np.maximum(s_mag, 10 ** (-15))
        mat = 20 * np.log10(s_mag / (2 ** 11)) # s_dbfs

        #if data.size != n_rows * n_cols:
        #    self.get_logger().error(f"Datos incompatibles con rows/cols: data={data.size}, rows*cols={n_rows*n_cols}")
        #    return

        #mat = data.reshape((n_rows, n_cols))

        # 2) Restar medición de fondo si está disponible
        if self.medicion_fondo is not None:
            try:
                if self.medicion_fondo.shape == mat.shape:
                    mat = mat - self.medicion_fondo
                elif self.medicion_fondo.ndim == 2 and self.medicion_fondo.shape[0] == 1 and self.medicion_fondo.shape[1] == cols:
                    mat = mat - self.medicion_fondo[0, :]
                elif self.medicion_fondo.ndim == 1 and self.medicion_fondo.shape[0] == cols:
                    mat = mat - self.medicion_fondo
                else:
                    self.get_logger().warn(f"medicion_fondo shape {self.medicion_fondo.shape} no calza; se omite resta.")
            except Exception as e:
                self.get_logger().warn(f"Error restando medición de fondo: {e!r}")

        # 3) Eje de frecuencia y r (distancia)
        freq = np.linspace(-self.sample_rate_hz/2.0, self.sample_rate_hz/2.0, cols, endpoint=False)
        r_all = self.freq_to_distance(freq)  # metros

        # 4) Filtrado por rango válido (>= 0 y opcionalmente <= max_range)
        min_r = float(self.get_parameter('min_range_m').value)
        max_r = float(self.get_parameter('max_range_m').value)
        valid_mask = (r_all >= min_r) & (r_all <= max_r)
        if not np.any(valid_mask):
            self.get_logger().warn("No hay bins de rango válidos tras el filtrado.")
            return

        mat = mat[:, valid_mask]
        r   = r_all[valid_mask]
        n_cols_valid = r.shape[0]

        # 6) CFAR por fila: índices de detección en range bins por cada steering angle
        total_ext  = self.cfar_guard_cells + self.cfar_reference_cells

        det_angle_idx = []
        det_range_idx = []

        for i in range(rows):
            mag = mat[i, :].astype(np.float64, copy=False)

            # Normalización fila a fila
            #mmin = float(np.min(mag))
            #mmax = float(np.max(mag))
            #if mmax > mmin:
            #    magn = (mag - mmin) / (mmax - mmin)
            #else:
            #    magn = np.zeros_like(mag)

            mag_ext = self.extend_with_means(mag, total_ext)
            _, targets = cfar(mag_ext,
                              num_guard_cells=self.cfar_guard_cells,
                              num_ref_cells=self.cfar_reference_cells,
                              bias=self.cfar_bias,
                              cfar_method=self.cfar_method)
            targets = np.ma.array(self.unpad(targets, total_ext),
                                  mask=self.unpad(targets.mask, total_ext))

            # obtener detecciones
            idxs = np.where(targets.mask)[0]
            if idxs.size > 0:
                det_angle_idx.append(np.full_like(idxs, i)) # se obtiene un vector con el idx del angulo en donde hubo deteccion
                det_range_idx.append(idxs) # se guarda el idx del bin donde hubo deteccion

        if len(det_angle_idx) == 0:
            self.get_logger().info("Sin detecciones en este mensaje.")
            return

        # obtiene vector con todas las detecciones tanto en distancia como en angulo
        det_angle_idx = np.concatenate(det_angle_idx)
        det_range_idx = np.concatenate(det_range_idx)

        # angulos absolutos
        # θ_abs = theta_pan + theta_beam (azimuth); phi_abs = tilt_deg (elevación)
        theta_abs_deg = - pan_deg + self.theta_beam_deg[det_angle_idx]
        phi_abs_deg = np.full_like(theta_abs_deg, tilt_deg, dtype=np.float64)

        theta_abs = np.deg2rad(theta_abs_deg)
        phi_abs = np.deg2rad(phi_abs_deg)

        # distancias de cada detección
        r_det = r[det_range_idx] # metros

        # conversión a (x,y,z) (z hacia arriba). Convención ENU típica:
        #x hacia el Este
        #y hacia el Norte
        #z hacia arriba
        # x = r cos(phi) sin(theta)
        # y = r cos(phi) cos(theta)
        # z = r sin(phi)
        cos_phi = np.cos(phi_abs)
        sin_phi = np.sin(phi_abs)
        sin_theta   = np.sin(theta_abs)
        cos_theta   = np.cos(theta_abs)

        x = r_det * cos_phi * sin_theta
        y = r_det * cos_phi * cos_theta
        z = r_det * sin_phi

        # publicar RadarCartesian
        out = RadarCartesian()
        # out.header = Header()
        # out.header.stamp = self.get_clock().now().to_msg()
        out.header = msg.header # reutilizar header del radar, mejor para sincronizar

        out.x = x.astype(np.float32).tolist()
        out.y = y.astype(np.float32).tolist()
        out.z = z.astype(np.float32).tolist()

        out.robot_pose_id = msg.robot_pose_id
        # Si tienes un campo opcional de intensidad y quieres llenarlo:
        # intens = mat[det_angle_idx, det_range_idx]
        # if hasattr(out, 'intensity'):
        #     out.intensity = intens.astype(np.float32).tolist()

        self.pub.publish(out)
        self.get_logger().info(f"Publicado /radar_cartesian con {len(out.x)} puntos "
                               f"(pan={pan_deg:.1f}°, tilt={tilt_deg:.1f}°)")

def main(args=None):
    rclpy.init(args=args)
    node = RadarSphericalToCartesian()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
