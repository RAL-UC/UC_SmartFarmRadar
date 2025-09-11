#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from radar_msg.msg import RadarData
import os
import time
from pathlib import Path
from radar_package.target_detection_dbfs import cfar # objetivos de deteccion
from radar_package.parametros import *

# Ruta por defecto
REPO_ROOT = Path.cwd() / "UC_SmartFarmRadar"
DEFAULT_OUTPUT = str(REPO_ROOT / "datos" / "medicion_fondo_cielo.npy")

class CerrarNode(Exception):
    """Cerrar nodo"""
    pass

class BackgroundCaptureNode(Node):
    """
    Suscribe a 'radar_data', acumula N capturas, promedia y guarda un archivo .npy.
    Parámetros ROS:
      - topic (str): nombre del tópico (default: 'radar_data')
      - captures (int): número de capturas de fondo a promediar (default: 10)
      - output_path (str): ruta del archivo .npy a guardar (default: DEFAULT_OUTPUT)
      - dtype (str): dtype NumPy para guardar (default: 'float64') o 32
    """
    def __init__(self):
        super().__init__('background_capture_node')

        # Declaración de parámetros (mismo estilo que tus otros nodos)
        self.topic = "/radar_data"
        self.declare_parameter('captures', REPEAT_CAPTURE)
        self.declare_parameter('output_path', DEFAULT_OUTPUT)
        self.np_data_type = 'float64'

        # Lectura de parámetros
        p = self.get_parameter
        self.n_captures = int(p('captures').get_parameter_value().integer_value)
        self.output_path = p('output_path').get_parameter_value().string_value

        if self.n_captures <= 0:
            raise ValueError("El parámetro 'captures' debe ser > 0")

        # subscripción
        self.create_subscription(RadarData, self.topic, self.listener_callback, 10)

        # acumuladores
        self.sum_accum = None
        self.count = 0
        self.shape_expected = None

        # Info inicial
        self.get_logger().info(
            f"Captura de {self.n_captures} mediciones de fondo"
        )

        # crear carpeta destino si no existe
        out_dir = os.path.dirname(self.output_path) or "."
        os.makedirs(out_dir, exist_ok=True)

    def listener_callback(self, msg: RadarData):
        # mostrar header
        self.get_logger().info(f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, id="{msg.header.frame_id}"')

        # Reconstruir matriz original [rows, cols]
        try:
            arr = np.array(msg.data, dtype=msg.dtype)  # respetamos el dtype que llega en el mensaje
            mat = arr.reshape((msg.rows, msg.cols))
        except Exception as e:
            self.get_logger().error(f"Error reconstruyendo la matriz desde /radar_data: {e}")
            return

        # inicializar forma y acumulador en la primera captura
        if self.shape_expected is None:
            self.shape_expected = mat.shape
            self.sum_accum = np.zeros(self.shape_expected, dtype=np.float64)  # usar float64 para acumular con precisión

        # validar forma consistente
        if mat.shape != self.shape_expected:
            self.get_logger().warn(f"Forma inconsistente {mat.shape} != {self.shape_expected}; omitiendo esta captura.")
            return

        # acumular
        self.sum_accum += mat
        self.count += 1
        self.get_logger().info(f"Captura {self.count}/{self.n_captures} acumulada.")

        # Si alcanzamos N, promediamos y guardamos
        if self.count >= self.n_captures:
            try:
                avg = (self.sum_accum / float(self.n_captures)).astype(self.np_data_type, copy=False) # avg

                cg = 1 # celdas de guarda
                cr = 15 # celdas de referencia
                b = 0 # valor bias
                m = "average" # metodo de calculo del umbral
                total_ext = cg + cr
                
                resultados = []
                for fila in avg:  
                    # extender bordes
                    mag_ext = self.extend_with_means(fila, total_ext)
                
                    # aplicar CFAR a la fila
                    thresh, targets = cfar(
                        mag_ext,
                        num_guard_cells=cg,
                        num_ref_cells=cr,
                        bias=b,
                        cfar_method=m
                    )
                
                    # quitar extensión
                    fila_procesada = self.unpad(thresh, total_ext)
                    resultados.append(fila_procesada)
                
                medicion_fondo = np.vstack(resultados)
                medicion_fondo = medicion_fondo.data

                np.save(self.output_path, medicion_fondo)
                self.get_logger().info(
                    f"[OK] Fondo promediado guardado en '{self.output_path}' "
                    f"(shape={medicion_fondo.shape}, dtype={medicion_fondo.dtype})"
                )
            except Exception as e:
                self.get_logger().error(f"Error al guardar '{self.output_path}': {e}")
            finally:
                # cerrar el nodo al terminar
                #rclpy.shutdown()
                raise CerrarNode("Cerrar nodo")
            

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
    node = BackgroundCaptureNode()
    try:
        rclpy.spin(node) # se queda escuchando callbacks
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Excepción no controlada: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
