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
        # Inicializar ventana de Matplotlib
        plt.ion() # Modo interactivo
        self.fig = None
        self.im = None
        self.im_det = None

        self.base_data = np.load(path_base_data) 

        # parámetros configurables desde línea de comandos o launch 
        self.declare_parameter('angle_min', -80) # grados
        self.declare_parameter('angle_max', 80) # grados
        self.declare_parameter('angle_step', 1) # grados

        # Leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        self.angles = np.arange(self.angle_min, self.angle_max+1, self.angle_step) # vector de angulos en recorrido

        # Funciones de conversión freq <-> range (eje inferior y superior)
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + (d * 2 * SLOPE / C)
        
        # figura para umbral CFAR, señal y detecciones vs distancia
        self.fig2 = None

        self.fig_accum = None
        self.im_accum = None

        self.total_joints = 13 # 180/15 + 1 -> desplazamiento de pantilt de 15 grados
        self.joint_counter = 0
        self.current_col = 0
        self.accumulated_map = None

    def listener_callback(self, msg: RadarData):
        # mostrar Header
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')

        # Reconstruir numpy array
        arr = np.array(msg.data, dtype=msg.dtype)
        n_steering_angle, n_bins = [msg.rows, msg.cols]
        mat = arr.reshape((n_steering_angle, n_bins))
        #mat = mat - self.base_data # banda base

        # Construir eje de frecuencia completo y corrimiento
        freq = np.linspace(-SAMPLE_RATE / 2, SAMPLE_RATE / 2, n_bins, endpoint=False)
        freq_step = freq[1] - freq[0] # SAMPLE_RATE/n_bins
        offset_idx = int(OFFSET / freq_step)

        # Convertir el eje de frecuencias a rango
        # Calcular el beat como diferencia con signal_freq y convertirlo a rango
        distance = self.freq_to_distance(freq)

        # Filtrar solo distancias >= 0
        valid_indices = np.where((distance >= 0))[0]

        # Desplazar datos en frecuencia y luego filtrar columnas
        rolled = np.roll(mat, -offset_idx, axis=1)
        filtered_data = rolled[:, valid_indices]

        freq = freq[valid_indices]
        distance = self.freq_to_distance(freq)

        detecciones = np.full((len(self.angles), len(freq)), 0) # np.nan

        for i, mag in enumerate(filtered_data):
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

            #if i == 80:
            #    self.grafico_2d(distance, mag, thresh, det_indices) # debug
        #self.grafico_3D(detecciones, self.angles, distance, filtered_data)
        
        # construccion de mapa completo
        if self.accumulated_map is None:
            # ancho suficiente para 13 matrices
            self.accumulated_map = np.full((detecciones.shape[0]* self.total_joints, detecciones.shape[1]), 0) # np.nan

        start_row = self.current_col # fila actual, se capturan las detecciones del steering angle para desplazamientos del pantilt
        end_row = start_row + detecciones.shape[0]

        # no pasarse del tamaño
        if end_row <= self.accumulated_map.shape[0]:
            self.accumulated_map[start_row:end_row, :] = detecciones
            self.current_col = end_row
            self.joint_counter += 1
        
        #self.grafico_mapa_acumulado(self.accumulated_map, distance)

        # llegado a 13 joints
        if self.joint_counter == self.total_joints:
            print("¡Se completaron los 13 joints!")

            # guardar imagen de detecciones
            #save_name = f"mapa_acumulado_{time.strftime('%Y%m%d_%H%M%S')}.npy"
            #np.save(save_name, self.accumulated_map)
            #print(f"Matriz acumulada guardada como {save_name}")
            
            # reseteo de valores
            self.accumulated_map = None
            self.joint_counter = 0
            self.current_col = 0

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
        
    # Función para recortar de vuelta
    def unpad(self, v, total_guard_ref):
        return v[total_guard_ref:-total_guard_ref]


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