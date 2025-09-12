#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Header

from radar_msg.msg import RadarData
from radar_msg.msg import RadarCartesian

from radar_package.target_detection_dbfs import cfar
from radar_package.parametros import *

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

        # suscriptor y publicador
        self.sub = self.create_subscription(RadarData, 'radar_data', self.cb_radar, 10)
        self.pub = self.create_publisher(RadarCartesian, 'radar_cartesian', 10)

        # Recursos: medición de fondo
        try:
            pkg_share = get_package_share_directory('radar_package')
            self.path_medicion_fondo = os.path.join(pkg_share, 'resource', 'medicion_fondo.npy')
            self.medicion_fondo = np.load(self.path_medicion_fondo) # shape esperada (rows, cols) o (1, cols)
            self.get_logger().info(f"Cargada medición de fondo desde: {self.path_medicion_fondo}")
        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar medición de fondo: {e!r}. Usando ceros.")
            self.medicion_fondo = None

        # barrido en azimuth theta_beam
        self.declare_parameter('angle_min_radar_beam', ANGLE_MIN_RADAR_BEAM)
        self.declare_parameter('angle_max_radar_beam', ANGLE_MAX_RADAR_BEAM)
        self.declare_parameter('angle_step_radar_beam', ANGLE_STEP_RADAR_BEAM)

        # parámetros CFAR
        self.declare_parameter('cfar_guard', CFAR_GUARD)
        self.declare_parameter('cfar_ref', CFAR_REF)
        self.declare_parameter('cfar_bias', CFAR_BIAS)
        self.declare_parameter('cfar_method', CFAR_METHOD)

        # filtro de distancias
        self.declare_parameter('min_range_m', MIN_RANGE_M)
        self.declare_parameter('max_range_m', MAX_RANGE_M)

        # leer parámetros
        p = self.get_parameter
        self.angle_min_radar_beam = p('angle_min_radar_beam').value
        self.angle_max_radar_beam = p('angle_max_radar_beam').value
        self.angle_step_radar_beam = p('angle_step_radar_beam').value

        self.cfar_guard = p('cfar_guard').value
        self.cfar_ref   = p('cfar_ref').value
        self.cfar_bias  = p('cfar_bias').value
        self.cfar_meth  = p('cfar_method').value
        
        self.win_funct = np.ones(GOOD_RAMP_SAMPLES, dtype=np.float64) # ventana rectangular
        #self.win_funct = np.blackman(GOOD_RAMP_SAMPLES) # ventana blackman -> posiblemente se deba considerar offset
        #self.win_funct = np.hamming(GOOD_RAMP_SAMPLES)
        self.sum_win_funct = np.sum(self.win_funct)

        # conversión frecuencia <-> distancia
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2.0 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + OFFSET + (d * 2.0 * SLOPE / C)  

        # construir vector de theta_beam
        self.theta_beam_deg = np.arange(self.angle_min_radar_beam, self.angle_max_radar_beam + 1, self.angle_step_radar_beam)
        self.get_logger().info(f"theta_beam_deg: {self.theta_beam_deg}")

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

    # ------------------ Callback principal ------------------

    def cb_radar(self, msg: RadarData):
        # 1) Reconstruir matriz (n_steer × n_bins)
        try:
            np_dtype = np.dtype(msg.dtype)
        except Exception:
            np_dtype = np.float64

        n_rows, n_cols = int(msg.rows), int(msg.cols)
        
        data_real = np.array(msg.data_real,  dtype=msg.dtype)
        data_real = data_real.reshape((n_rows, n_cols))
        data_imag = np.array(msg.data_imag,  dtype=msg.dtype)
        data_imag = data_imag.reshape((n_rows, n_cols))

        mat = data_real + 1j*data_imag
        #mat = mat.reshape((n_rows, n_cols))
        mat[:,:GOOD_RAMP_SAMPLES] = mat[:,:GOOD_RAMP_SAMPLES] * self.win_funct
        sp = np.fft.fftshift(np.abs(np.fft.fft(mat, axis=1)), axes=1)
        s_mag = sp / self.sum_win_funct
        s_mag = np.maximum(s_mag, 10 ** (-15))
        mat = 20 * np.log10(s_mag / (2 ** 11)) # s_dbfs

        #if data.size != n_rows * n_cols:
        #    self.get_logger().error(f"Datos incompatibles con rows/cols: data={data.size}, rows*cols={n_rows*n_cols}")
        #    return

        #mat = data.reshape((n_rows, n_cols))

        # 2) Restar medición de fondo si está disponible
        #if self.medicion_fondo is not None:
        #    try:
        #        if self.medicion_fondo.shape == mat.shape:
        #            mat = mat - self.medicion_fondo
        #        elif self.medicion_fondo.ndim == 2 and self.medicion_fondo.shape[0] == 1 and self.medicion_fondo.shape[1] == n_cols:
        #            mat = mat - self.medicion_fondo[0, :]
        #        elif self.medicion_fondo.ndim == 1 and self.medicion_fondo.shape[0] == n_cols:
        #            mat = mat - self.medicion_fondo
        #        else:
        #            self.get_logger().warn(f"medicion_fondo shape {self.medicion_fondo.shape} no calza; se omite resta.")
        #    except Exception as e:
        #        self.get_logger().warn(f"Error restando medición de fondo: {e!r}")

        # 3) Eje de frecuencia y r (distancia)
        freq = np.linspace(-SAMPLE_RATE/2.0, SAMPLE_RATE/2.0, n_cols, endpoint=False)
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
        total_ext  = self.cfar_guard + self.cfar_ref

        det_angle_idx = []
        det_range_idx = []

        for i in range(n_rows):
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
                              num_guard_cells=self.cfar_guard,
                              num_ref_cells=self.cfar_ref,
                              bias=self.cfar_bias,
                              cfar_method=self.cfar_meth)
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
        pan_deg  = msg.pan_deg
        tilt_deg = msg.tilt_deg

        theta_abs_deg = pan_deg + self.theta_beam_deg[det_angle_idx]
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
