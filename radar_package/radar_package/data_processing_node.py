#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from radar_msg.msg import RadarData
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *
from radar_msg.msg import RadarCartesian

# debo mejorar recurso de datos al infinito
path_base_data = "/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/infinito1.npy"

# nodo de procesamiento de datos
class RadarDataProcessing(Node):
    def __init__(self):
        super().__init__('radar_data_processing')
        self.create_subscription(RadarData, 'radar_data', self.listener_callback, 10)
        self.cart_pub = self.create_publisher(RadarCartesian, 'radar_cartesian', 10)

        self.base_data = np.load(path_base_data) # banda base
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

        self.freq_axis = freq[valid_indices] # eje frecuencias valido

        # atenuar valores iniciales
        row_means = np.mean(filtered_data, axis=1)
        filtered_data[:,:IDX_ATTENUATION] = row_means[:, np.newaxis]

        # tamaño de datos
        self.shape_data = [n_steering_angle, len(valid_indices)]

        # acumulacion de segmentos
        self.segmentos.append(filtered_data)
        self.joint_counter += 1

        # completados los N_MAPS
        if self.joint_counter == N_MAPS:
            self.get_logger().info(f"Se han unido los {N_MAPS} segmentos de captura")
            accumulated_map = np.vstack(self.segmentos)
            segmentos = accumulated_map.reshape((N_MAPS, self.shape_data[0], self.shape_data[1]))
            combine_map, global_angles = self.combine_radial_scans(segmentos) # combinacion de segmentos con interpolacion 
            detecciones = self.generate_detections(combine_map) # obtención de detecciones

            #self.get_logger().info(f"tamaño detecciones: {detecciones.shape}")

            angle_idxs, freq_idxs = np.nonzero(detecciones)

            self.get_logger().info(f"tamaño angle_idxs: {angle_idxs.shape}")
            self.get_logger().info(f"tamaño freq_idxs: {freq_idxs.shape}")
            theta = np.deg2rad(global_angles[angle_idxs]) # angulos a radianes
            r = self.freq_to_distance(self.freq_axis[freq_idxs])

            x = r * np.sin(theta)
            y = r * np.cos(theta)

            msg_cart = RadarCartesian()
            msg_cart.header = msg.header
            msg_cart.x = x.astype(np.float32).tolist()
            msg_cart.y = y.astype(np.float32).tolist()
            self.cart_pub.publish(msg_cart)
            self.get_logger().info(f"Publicado mapa cartesiano con ({len(x)}, {len(y)}) puntos")

            # guardar imagen de detecciones
            #save_name = f"mapa_acumulado_{time.strftime('%Y%m%d_%H%M%S')}.npy"
            #np.save(save_name, self.accumulated_map)
            #print(f"Matriz acumulada guardada como {save_name}")
            
            # reseteo de valores
            self.segmentos = []
            self.joint_counter = 0

    def extend_with_means(self, mag, total_guard_ref):
        """
        Extiende el vector mag agregando `total_guard_ref` celdas al inicio y al final,
        usando el promedio de las primeras y últimas `total_guard_ref` celdas reales
        """
        mean_start = np.mean(mag[:total_guard_ref])
        mean_end = np.mean(mag[-total_guard_ref:])

        # relleno
        pad_start = np.full(total_guard_ref, mean_start)
        pad_end = np.full(total_guard_ref, mean_end)

        # vector extendido
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
            
        return combine_map, global_angles

    def generate_detections(self, combine_map):
        # genera detecciones de forma binaria utilizando algoritmo cfar
        tamaño = combine_map.shape
        detecciones = np.full((tamaño[0], tamaño[1]), 0) # np.nan
        
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
    node = RadarDataProcessing()
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