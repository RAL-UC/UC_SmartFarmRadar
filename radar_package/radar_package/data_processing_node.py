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

        self.base_data = np.load(path_base_data) 
        self.ptu_angles = ANGLE0 + STEP_DEG_PTU * np.arange(N_MAPS) # lista de angulos del pantilt

        # parámetros configurables
        self.declare_parameter('angle_min', ANGLE_MIN)
        self.declare_parameter('angle_max', ANGLE_MAX)
        self.declare_parameter('angle_step', ANGLE_STEP)

        # leer parámetros
        p = self.get_parameter
        self.angle_min = p('angle_min').get_parameter_value().integer_value
        self.angle_max = p('angle_max').get_parameter_value().integer_value
        self.angle_step  = p('angle_step').get_parameter_value().integer_value

        # Funciones de conversión freq <-> distancia con compensacion de offset
        self.freq_to_distance = lambda f: (f - SIGNAL_FREQ - OFFSET) * C / (2 * SLOPE)
        self.distance_to_freq = lambda d: SIGNAL_FREQ + OFFSET + (d * 2 * SLOPE / C)

        self.shape_data = None
        
        self.segmentos = []
        self.joint_counter = 0

    def listener_callback(self, msg: RadarData):
        # mostrar Header
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')

        # reconstruir matriz original
        arr = np.array(msg.data, dtype=msg.dtype)
        n_steering_angle, n_bins = [msg.rows, msg.cols]
        mat = arr.reshape((n_steering_angle, n_bins))
        #mat = mat - self.base_data # banda base

        # eje de frecuencia completo
        freq = np.linspace(-SAMPLE_RATE / 2, SAMPLE_RATE / 2, n_bins, endpoint=False)
        # convertir el eje de frecuencias a distancia
        distance = self.freq_to_distance(freq)
        # filtrar solo distancias >= 0
        valid_indices = np.where((distance >= 0))[0]
        filtered_data = mat[:, valid_indices]
        #freq = freq[valid_indices]

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
            self.mosaico(accumulated_map)

            # guardar imagen de detecciones
            #save_name = f"mapa_acumulado_{time.strftime('%Y%m%d_%H%M%S')}.npy"
            #np.save(save_name, self.accumulated_map)
            #print(f"Matriz acumulada guardada como {save_name}")
            
            # reseteo de valores
            self.segmentos = []
            self.joint_counter = 0



        #distance = self.freq_to_distance(freq)
        #
        #
        #
        #
        #detecciones = np.full((n_steering_angle, len(freq)), 0) # np.nan
        #
        #for i, mag in enumerate(filtered_data):
        #    mag_min = np.min(mag)
        #    mag_max = np.max(mag)
        #
        #    if mag_max > mag_min:
        #        mag = (mag - mag_min) / (mag_max - mag_min)
        #    else:
        #        mag = np.zeros_like(mag)
        #    num_guard_cells = 5
        #    num_ref_cells = 15
        #    total_ext = num_guard_cells + num_ref_cells
        #    mag_ext = self.extend_with_means(mag, total_ext)
        #    thresh, targets = cfar(mag_ext, num_guard_cells=5, num_ref_cells=15, bias=0.1, cfar_method='average')
        #    thresh = self.unpad(thresh, total_ext)
        #    targets = np.ma.array(self.unpad(targets, total_ext), mask=self.unpad(targets.mask, total_ext))
        #    det_indices = np.where(targets.mask)[0] # posiciones no enmascaradas
        #    
        #    detecciones[i, det_indices] = 1 # mag[det_indices] o 255


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
    
    def mosaico(self, accumulated_map):
        half_fov = FOV / 2.0 # division del FOV
        angulos_seg = [np.linspace(c - half_fov, c + half_fov, self.shape_data[0]) for c in self.ptu_angles] # angulos que cubre cada segmento

         # ángulos globales unificados (sin repeticiones)
        all_angles = np.concatenate(angulos_seg)
        global_angles = np.unique(all_angles)

        mapa_mosaico = np.zeros((GLOBAL_ANGLES, self.shape_data[1])) # tamaño mapa global

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
                    mapa_mosaico[k, j] = vals[0]

                # para varios valores: media ponderada con pesos normalizados
                elif len(vals) > 1:
                    pesos = np.array(pesos)
                    vals  = np.array(vals)
                    pesos_norm = pesos / pesos.sum() # la suma debe ser igual a 1
                    mapa_mosaico[k, j] = np.dot(pesos_norm, vals)

                # cuando ningún segmento cubre el ángulo
                else:
                    mapa_mosaico[k, j] = 0
            
        return mapa_mosaico

        
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