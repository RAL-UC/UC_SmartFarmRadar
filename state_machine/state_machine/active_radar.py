#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from radar_msg.action import Beamform


class BeamformRepeat(Node):
    def __init__(self):
        super().__init__('radar_call_repeat')

        # Parámetros
        self.declare_parameter('angle_deg', 0)
        self.declare_parameter('repeat', -1)
        self.declare_parameter('wait_server_timeout_s', 10.0)

        self.angle_deg = int(self.get_parameter('angle_deg').value)
        self.repeat = int(self.get_parameter('repeat').value)
        self.wait_timeout = float(self.get_parameter('wait_server_timeout_s').value)

        self.infinite = (self.repeat <= 0)  # <=0 => infinito
        self.client = ActionClient(self, Beamform, 'radar_beamform')

        self.current_iter = 0
        rep_msg = "inf" if self.infinite else str(self.repeat)
        self.get_logger().info(f"Beamforming a {self.angle_deg}° se repetirá {rep_msg}")
        self.start()

    def start(self):
        if not self.client.wait_for_server(timeout_sec=self.wait_timeout):
            self.get_logger().error("Servidor de acción 'radar_beamform' no disponible.")
            rclpy.shutdown()
            return
        self.send_goal()

    def send_goal(self):
        if (not self.infinite) and (self.current_iter >= self.repeat):
            self.get_logger().info("Todas las repeticiones completadas.")
            rclpy.shutdown()
            return

        goal = Beamform.Goal()
        goal.angle_deg = self.angle_deg 

        self.current_iter += 1
        tag = f"[{self.current_iter}/{self.repeat}]" if not self.infinite else f"[{self.current_iter}]"
        self.get_logger().info(f"{tag} radar call with {self.angle_deg}° PTU")

        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, fb):
        msg = getattr(fb.feedback, 'status', 'ejecutando…')
        self.get_logger().info(f"[Feedback] {msg}")

    def goal_response_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Error al enviar goal: {e!r}")
            # en bucle infinito, reintenta; si no, termina
            if self.infinite:
                self.get_logger().warn("Reintentando…")
                self.send_goal()
            else:
                rclpy.shutdown()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rechazado por el servidor.")
            self.send_goal() # intenta siguiente iteración
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Error recibiendo resultado: {e!r}")
            if self.infinite:
                self.get_logger().warn("Reintentando…")
                self.send_goal()
            else:
                rclpy.shutdown()
            return

        ok = getattr(result, 'success', True)
        message = getattr(result, 'message', '')
        if ok:
            self.get_logger().info(f"Beamforming OK. {message}")
        else:
            self.get_logger().warn(f"Beamforming falló. {message}")

        # al terminar un goal → lanzar siguiente repetición
        self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node = BeamformRepeat()
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
