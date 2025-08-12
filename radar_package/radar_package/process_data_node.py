#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *
import time

# debo mejorar recurso de datos al infinito
path_base_data = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/infinito1.npy"

class RadarDataSubscriber(Node):
    def __init__(self):
        super().__init__('subscribe_radar_data')
        self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)
        # inicializar ventana de Matplotlib
        plt.ion() # modo interactivo
        self.fig = None
        self.im = None
        self.im_det = None

        self.base_data = np.load(path_base_data) # banda base

        # parámetros configurables
        self.declare_parameter('angle_min', ANGLE_MIN) # grados
        self.declare_parameter('angle_max', ANGLE_MAX) # grados
        self.declare_parameter('angle_step', ANGLE_STEP) # grados

        # leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        # Funciones de conversión freq <-> range (eje inferior y superior en graficos)
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + OFFSET + (d * 2 * SLOPE / C)
        
        # figura para umbral CFAR, señal y detecciones vs distancia
        self.fig2 = None

        # figura para acumulacion de las capturas
        self.fig_accum = None
        self.im_accum = None

        self.total_joints = N_MAPS # 180/15 + 1 -> desplazamiento de pantilt de 15 grados
        self.joint_counter = 0
        self.current_col = 0
        self.accumulated_map = None

    def listener_callback(self, msg: RadarData):
        # mostrar Header
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')

        # reconstruir matriz original
        arr = np.array(msg.data, dtype=msg.dtype)
        n_steering_angle, n_bins = [msg.rows, msg.cols]
        mat = arr.reshape((n_steering_angle, n_bins))
        #mat = mat - self.base_data # banda base

        # construir eje de frecuencia completo
        freq = np.linspace(-SAMPLE_RATE / 2, SAMPLE_RATE / 2, n_bins, endpoint=False)
        #freq_step = freq[1] - freq[0] # SAMPLE_RATE/n_bins
        #offset_idx = int(OFFSET / freq_step)

        # convertir el eje de frecuencias a rango
        distance = self.freq_to_distance(freq)
        # filtrar solo distancias >= 0
        valid_indices = np.where((distance >= 0))[0]
        # desplazar datos en frecuencia y luego filtrar columnas
        #rolled = np.roll(mat, -offset_idx, axis=1)
        #filtered_data = rolled[:, valid_indices]
        filtered_data = mat[:, valid_indices]

        # atenuar valores iniciales
        row_means = np.mean(filtered_data, axis=1)
        filtered_data[:,:IDX_ATTENUATION] = row_means[:, np.newaxis]

        self.shape_data = [n_steering_angle, len(valid_indices)]

        self.segmentos.append(filtered_data)
        self.joint_counter += 1

        # completados los N_MAPS
        if self.joint_counter == N_MAPS:
            self.get_logger().info(f"Se han unido los {N_MAPS} segmentos de captura")
            accumulated_map = np.vstack(self.segmentos)
            combine_map = self.combine_radial_scans(accumulated_map)
            self.generate_detections(self, combine_map)

            # guardar imagen de detecciones
            #save_name = f"mapa_acumulado_{time.strftime('%Y%m%d_%H%M%S')}.npy"
            #np.save(save_name, self.accumulated_map)
            #print(f"Matriz acumulada guardada como {save_name}")
            
            # reseteo de valores
            self.segmentos = []
            self.joint_counter = 0

            #if i == 80:
            #    self.grafico_2d(distance, mag, thresh, det_indices) # debug
        #self.grafico_3D(detecciones, self.angles, distance, filtered_data)
        #self.grafico_mapa_acumulado(self.accumulated_map, distance)

    # comparacion señal original con promedio CFAR
    def grafico_2d(self, distance, mag, thresh, det_indices):
        # figura2 para umbral CFAR, señal y detecciones vs distancia
        if self.fig2 is None:
            self.fig2, self.ax2 = plt.subplots(figsize=(6, 4))
            self.ax2.set_xlabel("Range [m]")
            self.ax2.set_ylabel("MinMax Normalized Magnitude")
            self.ax2.set_title("CFAR Threshold, Signal, and Detections vs. Distance")
            self.ax2.grid(True)
            self.line_sig, = self.ax2.plot([], [], label="FFT")
            self.line_thr, = self.ax2.plot([], [], label="CFAR Threshold")
            self.scatter2d = self.ax2.scatter([], [], c='r', s=20, label="Detections")
            self.ax2.legend(loc="upper right")

        # Señal original magnitud normalizada vs distancia
        self.line_sig.set_data(distance, mag)
        # Umbral CFAR vs distancia
        self.line_thr.set_data(distance, thresh)
        # Puntos de detección CFAR
        self.scatter2d.set_offsets(np.c_[distance[det_indices], mag[det_indices]])

        # Ajusta límites si han cambiado
        self.ax2.set_xlim(distance[0], distance[-1])
        self.ax2.set_ylim(0, 1) # normalizacion min-max

        # actualizar grafico
        self.fig2.canvas.draw_idle()
        self.fig2.canvas.flush_events()


    def grafico_3D(self, detecciones, angles, distance, filtered_data):
        detecciones = detecciones.T
        filtered_data = filtered_data.T # ahora [n_bins, n_steering_angle]
        extent = [angles[0], angles[-1], distance[0], distance[-1]]

        if self.fig is None:
            self.fig, (self.ax_mag, self.ax_det) = plt.subplots(1, 2, figsize=(12, 5))
            # FFT
            self.im = self.ax_mag.imshow(filtered_data, aspect='auto', origin='lower', extent=extent, cmap='jet')
            self.ax_mag.set_xlabel("Angle [°]"); self.ax_mag.set_ylabel("Range [m]")
            self.cbar_fft = self.fig.colorbar(self.im, ax=self.ax_mag, label="MinMax Normalized Magnitude")
            
            # Detecciones
            self.im_det = self.ax_det.imshow(detecciones, aspect='auto', origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
            self.ax_det.set_xlabel("Angle [°]"); self.ax_det.set_ylabel("Range [m]")
            self.ax_det.set_title("CFAR Detections Map")

        self.im_det.set_data(detecciones)

        # actualizar datos
        self.im.set_data(filtered_data)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
    
    def grafico_mapa_acumulado(self, accumulated_map, distance):
        # grafico de detecciones acumuladas en tiempo real
        data_acc = accumulated_map.T
        extent_acc = [0, accumulated_map.shape[0], distance[0], distance[-1]]

        if self.fig_accum is None:
            self.fig_accum, self.ax_accum = plt.subplots(figsize=(12, 6))
            self.ax_accum.set_xlabel("Capture by angle addressing")
            self.ax_accum.set_ylabel("Range [m]")
            self.ax_accum.set_title("Binary map of detections")

        if self.im_accum is None:
            # en la primera iteracion se crea la imagen
            self.im_accum = self.ax_accum.imshow(
                data_acc,
                aspect='auto',
                origin='lower',
                extent=extent_acc,
                cmap='gray',
                vmin=0,
                vmax=1
            )
        else:
            # actualizar datos
            self.im_accum.set_data(data_acc)
            self.im_accum.set_extent(extent_acc)
        
        self.fig_accum.canvas.draw_idle()
        self.fig_accum.canvas.flush_events()

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
        
    # recortar vector
    def unpad(self, v, total_guard_ref):
        return v[total_guard_ref:-total_guard_ref]
    
    def combine_radial_scans(self, accumulated_map):
        half_fov = FOV / 2.0 # division del FOV
        angulos_seg = [np.linspace(c - half_fov, c + half_fov, self.shape_data[0]) for c in self.ptu_angles] # angulos que cubre cada segmento

         # ángulos globales unificados (sin repeticiones)
        all_angles = np.concatenate(angulos_seg)
        global_angles = np.unique(all_angles)

        combine_map = np.zeros((GLOBAL_ANGLES, self.shape_data[1])) # mapa global

        # para cada frecuencia
        for j in range(self.shape_data[1]): # de 0 a 321 
            # para cada angulo en el mapa global
            for k, ga in enumerate(global_angles): # de -170 a 170
                pesos = []
                vals  = []
                # revisar cada segmento
                for i in range(N_MAPS): # de 0 a 12
                    ai = angulos_seg[i] # extrae los angulos de un segmento 
                    if ai[0] <= ga <= ai[-1]: # revisa si el angulo global esta dentro de los espacios del segmento
                        idx = np.argmax(ai == ga) # encontrar el indice del angulo global dentro del segmento
                        val = accumulated_map[i, idx, j] # extraer valor
                        # peso según cercanía al centro de direccionamiento ptu
                        dist_rel = abs(ga - self.ptu_angles[i]) / half_fov
                        w = max(0.0, 1.0 - dist_rel)
                        pesos.append(w)
                        vals.append(val)

                # para un solo valor: lo devolvemos tal cual
                if len(vals) == 1:
                    combine_map[k, j] = vals[0]

                # para varios valores: media ponderada con pesos normalizados
                elif len(vals) > 1:
                    pesos = np.array(pesos)
                    vals  = np.array(vals)
                    pesos_norm = pesos / pesos.sum() # la suma debe ser igual a 1
                    combine_map[k, j] = np.dot(pesos_norm, vals)

                # cuando ningún segmento cubre el ángulo
                else:
                    combine_map[k, j] = 0
            
        return combine_map

    def generate_detections(self, combine_map):
        # genera detecciones de forma binaria utilizando algoritmo cfar
        detecciones = np.full((self.shape_data[0], self.shape_data[1]), 0) # np.nan
        
        for i, mag in enumerate(combine_map):
            mag_min = np.min(mag)
            mag_max = np.max(mag)
        
            if mag_max > mag_min:
                mag = (mag - mag_min) / (mag_max - mag_min)
            else:
                mag = np.zeros_like(mag)
            num_guard_cells = 5
            num_ref_cells = 15
            total_ext = num_guard_cells + num_ref_cells
            mag_ext = self.extend_with_means(mag, total_ext)
            thresh, targets = cfar(mag_ext, num_guard_cells=5, num_ref_cells=15, bias=0.1, cfar_method='average')
            thresh = self.unpad(thresh, total_ext)
            targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
            det_indices = np.where(targets.mask)[0] # posiciones no enmascaradas
            
            detecciones[i, det_indices] = 1 # mag[det_indices] o 255

        return detecciones

def main(args=None):
    rclpy.init(args=args)
    node = RadarDataSubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)  # Escucha un poco y sigue
            plt.pause(0.001) # Permite actualizar la ventana matplotlib
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()