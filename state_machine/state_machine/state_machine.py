#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
from radar_msg.msg import RadarData
from radar_msg.action import PtuSweep, Beamform
from ral_bunker_msgs.action import NextPose

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self._ptu_client = ActionClient(self, PtuSweep, 'ptu_sweep')
        self._radar_client = ActionClient(self, Beamform, 'radar_beamform') 
        self._bunker_action_client = ActionClient(self, NextPose, 'next_pose')

        self._radar_pub = self.create_publisher(RadarData, 'radar_data', 10)

        # Listas de ángulos PAN/TILT emparejadas
        self.ptu_angles_pan = [90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90]
        self.ptu_angles_tilt = [0]*len(self.ptu_angles_pan)

        # Verificación de longitudes
        if len(self.ptu_angles_pan) != len(self.ptu_angles_tilt):
            self.get_logger().warn(
                f"Listas PAN({len(self.ptu_angles_pan)}) y TILT({len(self.ptu_angles_tilt)}) con longitudes distintas; "
                f"usaré el mínimo."
            )
        self._num_points = min(len(self.ptu_angles_pan), len(self.ptu_angles_tilt))

        self._idx = 0
        self._current_angle = None
        self._busy = False
        
        self.start_cycle()

    ############## PTU ###############
    def start_cycle(self):
        self._idx = 0
        self.get_logger().info(f"Inicio de ciclo PTU -> Radar. Ángulos: {self.ptu_angles}")
        self._command_ptu_for_current()

    def _command_ptu_for_current(self):
        if self._idx >= len(self._num_points):
            # Terminamos todos los ángulos ⇒ solicitar movimiento de bunker
            self.get_logger().info("Todos los ángulos completados → solicitando bunker next_pose")
            self.send_bunker_next_goal()
            return

        pan  = int(self.ptu_angles_pan[self._idx])
        tilt = int(self.ptu_angles_tilt[self._idx])

        self._current_pan = pan
        self._current_tilt = tilt
        self._busy = True

        goal = PtuSweep.Goal()
        # Importante: mandamos SOLO el ángulo actual
        goal.pan_deg  = pan
        goal.tilt_deg = tilt
        #goal.tolerance_steps = 0
        #goal.query_timeout_s = 8.0

        self.get_logger().info(f"[{self._idx+1}/{self._num_points}] PTU → pan={pan}°, tilt={tilt}°")
        self._ptu_client.wait_for_server()
        fut = self._ptu_client.send_goal_async(goal, feedback_callback=self._ptu_feedback_cb)
        fut.add_done_callback(self._ptu_goal_response_cb)

    def _ptu_feedback_cb(self, fb):
        f = fb.feedback
        pan  = getattr(f, 'current_pan_deg', None)
        tilt = getattr(f, 'current_tilt_deg', None)
        status = getattr(f, 'status', '')
        self.get_logger().info(f"[PTU] pan={pan}°, tilt={tilt}° | {status}")

    def _ptu_goal_response_cb(self, future):
        exc = future.exception()
        if exc:
            self._busy = False
            self.get_logger().error(f"Error al enviar goal PTU: {exc!r}")
            return
        gh = future.result()
        if gh is None or not gh.accepted:
            self._busy = False
            self.get_logger().warn("PTU goal rechazada")
            return
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._ptu_result_cb)

    def _ptu_result_cb(self, future):
        exc = future.exception()
        if exc:
            self._busy = False
            self.get_logger().error(f"Error recibiendo resultado PTU: {exc!r}")
            return
        
        res = future.result().result
        if not res.success:
            self._busy = False
            self.get_logger().warn(f"PTU movimiento falló: {res.message}")
            return
        
        # Al terminar el PTU en ese ángulo -> BEAMFORM de ese mismo ángulo
        pan = self._current_pan
        tilt = self._current_tilt
        self.get_logger().info(f"PTU OK (pan={pan}°, tilt={tilt}°) → Radar Beamform")
        self.start_beamforming(pan, tilt)

    ############## RADAR ###############
    def start_beamforming(self, pan_deg: int, tilt_deg: int):
        self._radar_client.wait_for_server()
        goal_bf = Beamform.Goal()
        goal_bf.pan_deg  = int(pan_deg)
        goal_bf.tilt_deg = int(tilt_deg)

        fut = self._radar_client.send_goal_async(goal_bf, feedback_callback=self._radar_feedback_cb)
        fut.add_done_callback(self._radar_goal_response_cb)

    def _radar_feedback_cb(self, fb):
        f = fb.feedback
        self.get_logger().info(f"[RADAR] {getattr(f, 'status', '')}")
    
    def _radar_goal_response_cb(self, future):
        exc = future.exception()
        if exc:
            self._busy = False
            self.get_logger().error(f"Error al enviar goal Radar Beamform: {exc!r}")
            return
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("Beamform goal rechazada")
            return
        gh.get_result_async().add_done_callback(self._radar_result_cb)

    def _radar_result_cb(self, future):
        exc = future.exception()
        if exc:
            self._busy = False
            self.get_logger().error(f"Error recibiendo resultado Radar Beamform: {exc!r}")
            return
        res = future.result().result
        if not res.success:
            self._busy = False
            self.get_logger().warn(f"Beamforming falló: {res.message}")
            return
        
        rd = getattr(res, 'radar_data', None)
        if rd is not None:
            self._radar_pub.publish(rd) 
            self.get_logger().info(
                f"Beamforming OK. RadarData recibido: {rd.rows}x{rd.cols} (dtype={rd.dtype})"
            )
        else:
            self.get_logger().warn("Beamforming OK, pero Result no trae 'radar_data'.")

        self._idx += 1
        self._busy = False
        self._command_ptu_for_current()

    ############## BUNKER ###############
    def send_bunker_next_goal(self):
        goal_msg = NextPose.Goal()
        goal_msg.go_to_next_pose = True

        self._bunker_action_client.wait_for_server()

        # ASYNC VERSION
        fut = self._bunker_action_client.send_goal_async(goal_msg)
        fut.add_done_callback(self.bunker_goal_response_callback)

    def bunker_goal_response_callback(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error al enviar goal bunker: {exc!r}")
            return
        
        goal_handle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_bunker_result_callback)

    def get_bunker_result_callback(self, future):
        #"""
        #This function is called when the bunker has finished
        #"""
        #self.current_index = 0
        #result = future.result().result
        #self.get_logger().info(f'Result: {result}')}

        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error recibiendo resultado bunker: {exc!r}")
            return
        res = future.result().result
        self.get_logger().info(f"Bunker listo ({res}) → reiniciando PTU sweep")
        # Loop: vuelve a iniciar el ciclo PTU
        self.start_cycle()


def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(state_machine)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        state_machine.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()