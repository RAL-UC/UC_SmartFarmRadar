#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor # permitir que un nodo de ROS 2 ejecute múltiples callbacks en paralelo, usando varios hilos (threads)
#from std_msgs.msg import Header
from radar_msg.msg import RadarData
from radar_msg.action import PtuSweep, RadarBeamform
from ral_bunker_msgs.action import NextPose
from std_srvs.srv import Empty # servicio que no tiene datos de entrada ni de salida -> sirve como disparador trigger

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.ptu_client = ActionClient(self, PtuSweep, 'ptu_sweep')
        self.radar_client = ActionClient(self, RadarBeamform, 'radar_beamform') 
        self.bunker_client = ActionClient(self, NextPose, 'next_pose')

        self.radar_pub = self.create_publisher(RadarData, 'radar_data', 10)

        #self._clear_cli = self.create_client(Empty, 'clear_map')

        # Listas de ángulos PAN/TILT emparejadas
        self.ptu_angles_pan = [90, 75, 60, 45, 30, 15, 0, -15, -30, -45, -60, -75, -90]
        self.ptu_angles_tilt = [-20, -15, -10, -5, 0]
        #self.ptu_angles_tilt = [0]*len(self.ptu_angles_pan)

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

        self.bunker_pose_id = 0 # id posicion del bunker
        
        self.start_cycle()

    ############## PTU ###############
    def current_angles(self):
        """[pan, tilt]"""
        return int(self.ptu_angles_pan[self.i_pan]), int(self.ptu_angles_tilt[self.i_tilt])
    
    def advance_indices(self):
        """primero recorre todos los tilt para el pan actual luego avanza pan"""
        self.count += 1
        self.i_tilt += 1
        if self.i_tilt >= self.N_tilt:
            self.i_tilt = 0
            self.i_pan += 1
    
    # recorrido terminado
    def grid_done(self):
        return self.count >= self.total or self.i_pan >= self.N_pan

    def start_cycle(self):
        self.i_pan = 0
        self.i_tilt = 0
        self.count = 0

        self.get_logger().info(f"Inicio de ciclo PTU - Radar")
        self.command_ptu_for_current()

    def command_ptu_for_current(self):
        if self.grid_done():
            self.get_logger().info("Grilla PAN TILT completa -> solicitando bunker next_pose")
            self.send_bunker_next_goal()
            return

        pan, tilt = self.current_angles()
        self.current_pan = pan
        self.current_tilt = tilt
        self.busy = True

        goal = PtuSweep.Goal()
        # mandamos SOLO el ángulo actual
        goal.pan_deg  = pan
        goal.tilt_deg = tilt

        self.get_logger().info(
            f"[{self.count+1}/{self.total}] PTU -> pan={pan}°, tilt={tilt}° | pose_id={self.bunker_pose_id}")
        

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
        
        # Al terminar el PTU en ese ángulo -> BEAMFORM de ese mismo ángulo
        pan = self.current_pan
        tilt = self.current_tilt
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
            try:
                rd.bunker_pose_id = int(self.bunker_pose_id)
                rd.pan_deg = int(self.current_pan)
                rd.tilt_deg = int(self.current_tilt)
                self.get_logger().info(f"Beamforming OK. RadarData recibido: {rd.rows}x{rd.cols} (dtype={rd.dtype})"
                )
            except Exception as e:
                self.get_logger().warn(f"No pude setear metadatos en RadarData: {e!r}")
            
            self.radar_pub.publish(rd)
            #self.get_logger().info(
            #    f"Beamforming OK. RadarData recibido: {rd.rows}x{rd.cols} (dtype={rd.dtype}) "
            #    f"[pose_id={self.bunker_pose_id}"
            #    f"pan={self.current_pan}, tilt={self.current_tilt}]"
            #)
        else:
            self.get_logger().warn("Beamforming OK, pero Result no trae 'radar_data'.")

        self.advance_indices()
        self.busy = False
        self.command_ptu_for_current()

    ############## BUNKER ###############
    def send_bunker_next_goal(self):
        goal_msg = NextPose.Goal()
        goal_msg.go_to_next_pose = True
        self.bunker_client.wait_for_server()
        # control asyncrono de evento/acciones
        fut = self.bunker_client.send_goal_async(goal_msg) # enviar objetivo
        fut.add_done_callback(self.bunker_goal_response_callback) # registrar callback

    def bunker_goal_response_callback(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error al enviar goal bunker: {exc!r}")
            return
        
        goal_handle = future.result() # GoalHandle manejo de objetivo

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.get_bunker_result_callback) # obtener resultado y registrar callback

    def get_bunker_result_callback(self, future):
        exc = future.exception()
        if exc:
            self.get_logger().error(f"Error recibiendo resultado bunker: {exc!r}")
            return
        res = future.result().result
        self.get_logger().info(f"Bunker listo ({res}) -> reiniciando PTU sweep")

        # reiniciar loop con nueva pose_id
        self.bunker_pose_id += 1
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