#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# mensajes y acciones
from radar_msg.msg import Ptu, MobileRobot, RadarData
from radar_msg.action import PtuSweep, RadarBeamform

from rclpy.action import ActionClient


class PtuRadarScan(Node):

    def __init__(self):
        super().__init__("ptu_radar_scan")

        # vector de angulos
        self.pan_list = [-30, -15, 0, 15, 30]
        self.tilt_list = [0] * len(self.pan_list) # mismo largo que pan_list

        # estado interno
        self.i_pan = 0
        self.i_tilt = 0
        self.count = 0
        self.total = len(self.pan_list) * len(self.tilt_list)
        self.N_pan  = len(self.ptu_angles_pan)
        self.N_tilt = len(self.ptu_angles_tilt)

        self.current_pan = None
        self.current_tilt = None
        self.busy = False # ocupado

        self.robot_pose_id = 0

        # Publishers
        self.pan_tilt_pub = self.create_publisher(Ptu, "/ptu_data", 10)
        self.robot_pub = self.create_publisher(MobileRobot, "/mobilerobot_data", 10)
        self.radar_pub = self.create_publisher(RadarBeamform.Result, "/radar_data", 10)

        # Subscribers
        self.trigger_sub = self.create_subscription(Bool, "/start_scan", self.trigger_cb, 10)

        # Action Clients
        self.ptu_client = ActionClient(self, PtuSweep, "/ptu_sweep")
        self.radar_client = ActionClient(self, RadarBeamform, "/radar_beamform")

        self.get_logger().info("PTU-Radar Scan Node listo")

    # TRIGGER
    def trigger_cb(self, msg: Bool):
        if not msg.data:
            return

        if self.busy:
            self.get_logger().warn("Scan ya en ejecución")
            return

        self.i_pan = 0
        self.i_tilt = 0
        self.count = 0
        self.busy = True

        self.get_logger().info("Trigger recibido → inicio de ciclo PTU-Radar")
        self.command_ptu_for_current()

    # recorrido terminado
    def grid_done(self):
        return self.count >= self.total
    
    # PTU
    def command_ptu_for_current(self):
        if self.grid_done():
            self.get_logger().info("Grilla PAN TILT completa -> solicitando robot movil next_pose")
            self.robot_next_goal()
            return

        pan = self.pan_list[self.i_pan]
        tilt = self.tilt_list[self.i_tilt]

        self.current_pan = pan
        self.current_tilt = tilt

        goal = PtuSweep.Goal()
        goal.target_ptu.pan_deg = int(pan)
        goal.target_ptu.tilt_deg = int(tilt)

        self.get_logger().info(
            f"[{self.count+1}/{self.total}] PTU → pan={pan}°, tilt={tilt}°"
        )

        self.ptu_client.wait_for_server()
        future = self.ptu_client.send_goal_async(goal)
        future.add_done_callback(self.ptu_goal_response_cb)

    def ptu_goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("PTU goal rechazado")
            self.busy = False
            return

        goal_handle.get_result_async().add_done_callback(self.ptu_result_cb)

    def ptu_result_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error recibiendo resultado PTU: {exc!r}")
            return

        res = future.result().result
        if not res.success:
            self.busy = False
            self.get_logger().error(f"PTU falló: {res.message}")
            return

        pan = self.current_pan
        tilt = self.current_tilt

        # Evento PTU
        pt = Ptu()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.pan_deg = int(pan)
        pt.tilt_deg = int(tilt)
        self.pan_tilt_pub.publish(pt)

        self.get_logger().info(
            f"PTU OK (pan={pan}°, tilt={tilt}°) → Radar Beamform"
        )

        self.start_beamforming(pan, tilt)

    # RADAR
    def start_beamforming(self, pan, tilt):
        goal = RadarBeamform.Goal()
        goal.pan_deg = int(pan)
        goal.tilt_deg = int(tilt)

        self.radar_client.wait_for_server()
        future = self.radar_client.send_goal_async(goal)
        future.add_done_callback(self.radar_goal_response_cb)

    def radar_goal_response_cb(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Radar goal rechazado")
            self.busy = False
            return

        goal_handle.get_result_async().add_done_callback(self.radar_result_cb)

    def radar_result_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error recibiendo resultado Radar: {exc!r}")
            return

        res = future.result().result
        if not res.success:
            self.busy = False
            self.get_logger().error(f"Radar falló: {res.message}")
            return

        # Publicar radar (evento)
        radar_data = res.radar_data
        radar_data.header.stamp = self.get_clock().now().to_msg()
        radar_data.robot_pose_id = int(self.robot_pose_id)
        self.radar_pub.publish(radar_data)

        # Avanzar índice
        self.count += 1
        self.i_tilt += 1
        if self.i_tilt >= len(self.tilt_list):
            self.i_tilt = 0
            self.i_pan += 1

        self.command_ptu_for_current()

    def robot_next_goal(self):
        self.robot_pose_id += 1

        # Publicar estado del robot - evento de cambio de pose
        rm = MobileRobot()
        rm.header.stamp = self.get_clock().now().to_msg()
        rm.robot_pose_id = int(self.robot_pose_id)
        self.robot_pub.publish(rm)

# MAIN
def main(args=None):
    rclpy.init(args=args)
    node = PtuRadarScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()