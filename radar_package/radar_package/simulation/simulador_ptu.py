#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

from radar_msg.action import PtuSweep  # ajusta el import si está en otro paquete
#from radar_package.parametros import *

class PtuSimServer(Node):
    """
    Servidor de acción mínimo para ptu_sweep.
    - Acepta cualquier ángulo dentro del rango.
    - Responde inmediatamente con success=True.
    - Sin retardos ni simulación de movimiento.
    """

    def __init__(self):
        super().__init__('ptu_sim')

        # parámetros
        # ptu.yaml
        self.declare_parameter('ptu_pan_min_deg', -90)
        self.declare_parameter('ptu_pan_max_deg', 90)
        self.declare_parameter('ptu_pan_step_deg', 15)
        self.declare_parameter('ptu_tilt_min_deg', -20)
        self.declare_parameter('ptu_tilt_max_deg', 20)
        self.declare_parameter('ptu_tilt_step_deg', 5)

        # valores
        # ptu.yaml
        self.ptu_pan_min_deg  = self.get_parameter("ptu_pan_min_deg").value
        self.ptu_pan_max_deg = self.get_parameter("ptu_pan_max_deg").value
        self.ptu_pan_step_deg  = self.get_parameter("ptu_pan_step_deg").value
        self.ptu_tilt_min_deg  = self.get_parameter("ptu_tilt_min_deg").value
        self.ptu_tilt_max_deg = self.get_parameter("ptu_tilt_max_deg").value
        self.ptu_tilt_max_deg  = self.get_parameter("ptu_tilt_step_deg").value

        # Servidor de acción
        self._server = ActionServer(
            self,
            PtuSweep,
            'ptu_sweep',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb
        )

        self.get_logger().info(
            f"[PTU SIM] Rango pan=[{self.angle_min_ptu_pan},{self.angle_max_ptu_pan}]°, "
            f"tilt=[{self.angle_min_ptu_tilt},{self.angle_max_ptu_tilt}]°"
        )

    def goal_cb(self, goal_request):
        pan  = int(goal_request.pan_deg)
        tilt = int(goal_request.tilt_deg)

        if not (self.angle_min_ptu_pan <= pan <= self.angle_max_ptu_pan):
            self.get_logger().warn(f"Goal rechazado: pan={pan}° fuera de rango.")
            return GoalResponse.REJECT
        if not (self.angle_min_ptu_tilt <= tilt <= self.angle_max_ptu_tilt):
            self.get_logger().warn(f"Goal rechazado: tilt={tilt}° fuera de rango.")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    async def execute_cb(self, goal_handle):
        pan  = int(goal_handle.request.pan_deg)
        tilt = int(goal_handle.request.tilt_deg)

        # Completar inmediatamente
        goal_handle.succeed()
        result = PtuSweep.Result()
        result.success = True
        result.message = f"Simulado: PTU en pan={pan}°, tilt={tilt}°"
        self.get_logger().info(f"[PTU SIM] {result.message}")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PtuSimServer()
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
