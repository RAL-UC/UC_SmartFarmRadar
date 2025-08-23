import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from radar_msg.action import PtuSweep, Beamform
from ral_bunker_msgs.action import NextPose

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self._ptu_client = ActionClient(self, PtuSweep, 'ptu_sweep')
        self._radar_client = ActionClient(self, Beamform, 'radar_beamform') 
        self._bunker_action_client = ActionClient(self, NextPose, 'next_pose')


        self.ptu_angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        self._idx = 0
        self._current_angle = None
        self._busy = False
        
        self.start_cycle()

    ############## PTU ###############
    def start_cycle(self):
        self._idx = 0
        self.get_logger().info(f"Inicio de ciclo PTU→Radar. Ángulos: {self.ptu_angles}")
        self._command_ptu_for_current()

    def _command_ptu_for_current(self):
        if self._idx >= len(self.ptu_angles):
            # Terminamos todos los ángulos ⇒ solicitar movimiento de bunker
            self.get_logger().info("Todos los ángulos completados → solicitando bunker next_pose")
            self.send_bunker_next_goal()
            return

        angle = self.ptu_angles[self._idx]
        self._current_angle = angle
        self._busy = True

        goal = PtuSweep.Goal()
        # Importante: mandamos SOLO el ángulo actual
        goal.angles_deg = angle
        goal.tolerance_steps = 0
        goal.query_timeout_s = 8.0

        self.get_logger().info(f"[{self._idx+1}/{len(self.ptu_angles)}] PTU → {angle}°")
        self._ptu_client.wait_for_server()
        fut = self._ptu_client.send_goal_async(goal, feedback_callback=self._ptu_feedback_cb)
        fut.add_done_callback(self._ptu_goal_response_cb)

    def _ptu_feedback_cb(self, fb):
        f = fb.feedback
        self.get_logger().info(f"[PTU] ang={f.current_angle}° | {f.status}")

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
        
        # Al terminar el PTU en ese ángulo → BEAMFORM de ese mismo ángulo
        angle = self._current_angle
        self.get_logger().info(f"PTU OK en {angle}° -> Radar Beamform")
        self.start_beamforming(angle)

    ############## RADAR ###############
    def start_beamforming(self, angle_deg: int):
        self._radar_client.wait_for_server()
        goal_bf = Beamform.Goal()
        goal_bf.angle_deg = angle_deg

        fut = self._radar_client.send_goal_async(goal_bf, feedback_callback=self._radar_feedback_cb)
        fut.add_done_callback(self._radar_goal_response_cb)

    def _radar_feedback_cb(self, fb):
        f = fb.feedback
        self.get_logger().info(f"[RADAR] {f.status}")
    
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

        self.get_logger().info("Beamforming OK.")
        self._idx += 1
        self._busy = False
        self._command_ptu_for_current()

    ############## BUNKER ###############
    def send_bunker_next_goal(self):
        goal_msg = NextPose.Goal()
        goal_msg.go_to_next_pose = True

        self._bunker_action_client.wait_for_server()

        # ASYNC VERSION
        self._send_bunker_goal_future = self._bunker_action_client.send_goal_async(goal_msg)
        self._send_bunker_goal_future.add_done_callback(self.bunker_goal_response_callback)

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

        self._get_bunker_result_future = goal_handle.get_result_async()
        self._get_bunker_result_future.add_done_callback(self.get_bunker_result_callback)

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
        self.start_ptu_cycle()


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