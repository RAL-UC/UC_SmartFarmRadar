#!/usr/bin/env python3
import os
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Header
from radar_msg.msg import RadarData
from radar_msg.action import Beamform

REPO_ROOT = Path.cwd() / "UC_SmartFarmRadar"
DEFAULT_OUTPUT = str(REPO_ROOT / "datos" / "medicion_fondo.npy")

class RadarSimServer(Node):
    def __init__(self):
        super().__init__('radar_sim')

        # Parám: ruta del .npy (usa absoluta para mantenerlo simple)
        if not os.path.isfile(DEFAULT_OUTPUT):
            raise FileNotFoundError(f"No se encontró el .npy en: {DEFAULT_OUTPUT}")

        #self.get_logger().info(f"Cargando base desde: {DEFAULT_OUTPUT}")
        base = np.load(DEFAULT_OUTPUT) # base 2D: (rows, cols)
        #row_idx = 80
        #base = base[row_idx, np.newaxis, :] # shape (1, N)
        self.dtype_str = "float64" # coincide con tu RadarData.msg
        base = base.astype(self.dtype_str, copy=False)
        #if base.ndim != 2:
        #    raise RuntimeError(f"medicion_fondo.npy debe ser 2D. Shape={base.shape} | dtype={base.dtype}")

        # dimensiones y version aplanada
        self.rows = int(base.shape[0])
        self.cols = int(base.shape[1])
        self.data_flat = base.ravel().tolist()

        # Servidor de acción
        self.server = ActionServer(
            self,
            Beamform,
            'radar_beamform',
            execute_callback=self.execute_cb
        )


        self.get_logger().info(
            f"[RADAR SIM] listo. Shape base: {self.rows}x{self.cols} | dtype={self.dtype_str}"
        )

    async def execute_cb(self, goal_handle):
        # Ignoramos el ángulo para mantenerlo minimalista
        # angle = int(goal_handle.request.angles_deg)
        pan  = int(goal_handle.request.pan_deg)
        tilt = int(goal_handle.request.tilt_deg)

        # Empaquetar RadarData
        msg = RadarData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'radar_sim' # etiqueta simple
        msg.rows = self.rows
        msg.cols = self.cols
        msg.dtype = self.dtype_str
        msg.data = self.data_flat

        # Completar acción
        goal_handle.succeed()
        result = Beamform.Result()
        result.success = True
        result.message = "Simulación OK"
        result.radar_data = msg
        return result


def main(args=None):
    rclpy.init(args=args)
    node = RadarSimServer()
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
