#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor # permitir que un nodo de ROS 2 ejecute múltiples callbacks en paralelo, usando varios hilos (threads)
from std_msgs.msg import Header
from radar_msg.msg import Ptu, MobileRobot, RadarData
from radar_msg.action import PtuSweep, RadarBeamform
from ral_bunker_msgs.action import NextPose
#from std_srvs.srv import Empty # servicio que no tiene datos de entrada ni de salida -> sirve como disparador trigger

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.ptu_client = ActionClient(self, PtuSweep, 'ptu_sweep')
        self.radar_client = ActionClient(self, RadarBeamform, 'radar_beamform') 
        self.robot_client = ActionClient(self, NextPose, 'next_pose')

        self.radar_pub = self.create_publisher(RadarData, 'radar_data', 10)
        self.pan_tilt_pub = self.create_publisher(Ptu, 'ptu_data', 10)
        self.robot_pub   = self.create_publisher(MobileRobot, 'mobilerobot_data', 10)

        #self._clear_cli = self.create_client(Empty, 'clear_map')

        # Listas de ángulos PAN/TILT emparejadas
        self.ptu_angles_pan = [90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90]
        #self.ptu_angles_tilt = [-20, -15, -10, -5, 0]
        self.ptu_angles_tilt = [0]*len(self.ptu_angles_pan)

        self.N_pan  = len(self.ptu_angles_pan)
        self.N_tilt = len(self.ptu_angles_tilt)
        self.i_pan = 0
        self.i_tilt = 0
        self.count = 0
        self.total = self.N_pan * self.N_tilt

        # Verificación de longitudes
        #if len(self.ptu_angles_pan) != len(self.ptu_angles_tilt):
        #    self.get_logger().warn(
        #        f"Listas PAN({len(self.ptu_angles_pan)}) y TILT({len(self.ptu_angles_tilt)}) con longitudes distintas; "
        #        f"usaré el mínimo."
        #    )
        #self._num_points = min(len(self.ptu_angles_pan), len(self.ptu_angles_tilt))

        self.current_pan = None
        self.current_tilt = None
        self.busy = False # ocupacion de la rutina

        self.robot_pose_id = 0 # id posicion del robot
        
        self.start_cycle()

    ############## PTU ###############
    def advance_indices(self):
        """primero recorre todos los tilt para el pan actual luego avanza pan"""
        self.count += 1
        self.i_tilt += 1
        if self.i_tilt >= self.N_tilt:
            self.i_tilt = 0
            self.i_pan += 1
    
    # recorrido terminado
    def grid_done(self):
        return self.count >= self.total

    def start_cycle(self):
        self.i_pan = 0
        self.i_tilt = 0
        self.count = 0

        self.get_logger().info(f"Inicio de ciclo PTU - Radar")
        self.command_ptu_for_current()

    def command_ptu_for_current(self):
        if self.grid_done():
            self.get_logger().info("Grilla PAN TILT completa -> solicitando robot movil next_pose")
            self.send_robot_next_goal()
            return

        pan = self.pan_list[self.i_pan]
        tilt = self.tilt_list[self.i_tilt]
        self.current_pan = pan
        self.current_tilt = tilt
        self.busy = True

        goal = PtuSweep.Goal()
        # mensaje PTU dentro del Goal
        #goal.target_ptu.header.stamp = self.get_clock().now().to_msg()
        goal.target_ptu.pan_deg  = int(pan)
        goal.target_ptu.tilt_deg = int(tilt)

        self.get_logger().info(
            f"[{self.count+1}/{self.total}] PTU -> pan={pan}°, tilt={tilt}° | pose_id={self.robot_pose_id}")
        

        self.ptu_client.wait_for_server() # esperar servidor
        # enviar objetivo y se registra el callback de retroalimentacion
        fut = self.ptu_client.send_goal_async(goal, feedback_callback=self.ptu_feedback_cb) # rutina asincrona
        fut.add_done_callback(self.ptu_goal_response_cb)

    def ptu_feedback_cb(self, fb):
        f = fb.feedback
        self.get_logger().info(f"[PTU] pan={getattr(f,'current_pan_deg',None)}°, "
                               f"tilt={getattr(f,'current_tilt_deg',None)}° | {getattr(f,'status','')}")

    def ptu_goal_response_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error al enviar goal PTU: {exc!r}")
            return
        gh = future.result()
        if gh is None or not gh.accepted:
            self.busy = False
            self.get_logger().warn("PTU goal rechazada")
            return
        # obtener resultado y se registra el callback para cuando se completa
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self.ptu_result_cb)

    def ptu_result_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error recibiendo resultado PTU: {exc!r}")
            return
        
        res = future.result().result
        if not res.success:
            self.busy = False
            self.get_logger().warn(f"PTU movimiento falló: {res.message}")
            return
        
        # PTU llegó correctamente a la posición objetivo
        pan = self.current_pan
        tilt = self.current_tilt

        # Publicar estado PTU - evento de cambio
        pt = Ptu()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.pan_deg = int(pan)
        pt.tilt_deg = int(tilt)
        self.pan_tilt_pub.publish(pt)
        
        # Al terminar el PTU en ese ángulo -> BEAMFORM de ese mismo ángulo
        self.get_logger().info(f"PTU OK (pan={pan}°, tilt={tilt}°) -> Radar Beamform")
        self.start_beamforming(pan, tilt)

    ############## RADAR ###############
    def start_beamforming(self, pan_deg: int, tilt_deg: int):
        self.radar_client.wait_for_server()
        goal_bf = RadarBeamform.Goal()
        goal_bf.pan_deg  = int(pan_deg)
        goal_bf.tilt_deg = int(tilt_deg)

        fut = self.radar_client.send_goal_async(goal_bf, feedback_callback=self.radar_feedback_cb)
        fut.add_done_callback(self.radar_goal_response_cb)

    def radar_feedback_cb(self, fb):
        self.get_logger().info(f"[RADAR] {getattr(fb.feedback, 'status', '')}")
    
    def radar_goal_response_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error al enviar goal Radar Beamform: {exc!r}")
            return
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("Beamform goal rechazada")
            return
        gh.get_result_async().add_done_callback(self.radar_result_cb)

    def radar_result_cb(self, future):
        exc = future.exception()
        if exc:
            self.busy = False
            self.get_logger().error(f"Error recibiendo resultado Radar Beamform: {exc!r}")
            return
        res = future.result().result
        if not res.success:
            self.busy = False
            self.get_logger().warn(f"Beamforming falló: {res.message}")
            return
        
        rd = getattr(res, 'radar_data', None)
        if rd is not None:
            self.get_logger().warn("Beamforming OK, pero sin radar_data")
            return
        
        # timestamp comun
        stamp = self.get_clock().now().to_msg()

        # RadarData
        # ---- RadarData ----
        rd.header.stamp = stamp
        self.radar_pub.publish(rd)

        self.advance_indices()
        self.busy = False
        self.command_ptu_for_current()

    ############## Robot Movil ###############
    def send_robot_next_goal(self):
        goal_msg = NextPose.Goal()
        goal_msg.go_to_next_pose = True
        self.robot_client.wait_for_server()
        # control asyncrono de evento/acciones
        fut = self.robot_client.send_goal_async(goal_msg) # enviar objetivo
        fut.add_done_callback(self.robot_goal_response_callback) # registrar callback

    def robot_goal_response_callback(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error al enviar goal robot: {exc!r}")
            return
        
        goal_handle = future.result() # GoalHandle manejo de objetivo

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.robot_result_callback) # obtener resultado y registrar callback

    def robot_result_callback(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error recibiendo resultado robot: {exc!r}")
            return
        
        res = future.result().result
        self.get_logger().info(f"Robot listo ({res}) -> reiniciando PTU sweep")

        # reiniciar loop con nueva pose_id
        self.robot_pose_id += 1

        # Publicar estado del robot - evento de cambio de pose
        rm = MobileRobot()
        rm.header.stamp = self.get_clock().now().to_msg()
        rm.robot_pose_id = int(self.robot_pose_id)
        self.robot_pub.publish(rm)

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