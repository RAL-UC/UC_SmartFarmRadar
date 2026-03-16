#!/usr/bin/env python3
import time # manejo del tiempo
import re # manejo de strings
from threading import Event # sincronizacion entre hilos

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from radar_msg.action import PtuSweep
#import asyncio # corrutinas

# convencion RADAR
# positivo derecha
# negativo izquierda

# convención PTU
# negativo derecha
# positivo izquierda

# limites minimo y maximo de cada grado de libertad
# paneo
#PN * -> -3090 -> - 158.91423666666665
#PX * -> 3090 -> 158.91423666666665

# tilteo
#TN * -> -907 -> -46.645699888888885 hacia delante
#TX * -> 604 -> 31.062847555555553 hacia atras

# misma resolucion en ambas direcciones
# 90, 75, 60, 45, 30, 15, 0
# 1750, 1361, 1167, 875, 583, 292, 0

# conversion de metricas
def grados_a_pasos(grados: int) -> int:
    """Convierte grados a pasos para PTU-C46 (185.1428 arcsec/paso ≈ 0.0514285°)."""
    resolucion_grados = 185.1428 / 3600 # ≈ 0.0514285°
    pasos = round(grados / resolucion_grados)
    return pasos

def pasos_a_grados(pasos: int) -> int:
    """Convierte pasos a grados para PTU-C46 (185.1428 arcsec/paso ≈ 0.0514285°)."""
    resolucion_grados = 185.1428 / 3600 # ≈ 0.0514285°
    grados = round(pasos * resolucion_grados)
    return grados

class PtuRoutineNode(Node):
    def __init__(self):
        super().__init__('ptu_node')
        # topicos
        self.cmd_pub = self.create_publisher(String, '/ptu_cmd', 10) # publicar comandos
        self.rx_sub = self.create_subscription(String, '/ptu_response', self.rx_cb, 50) # respuestas del PTU

        # objetivos predefinidos
        #self.ptu_angles = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
        #self.current_index = 0

        self.active_goal = None # objetivo
        self.done_evt = Event() # control de eventos
        #self.cancel_evt = Event()
        self.success = False # estado de evento
        self.final_message = "OK" # feedback msg

        # objetivos en pasos
        self.target_pan_steps  = None
        self.target_tilt_steps = None

        # ultima posicion en pasos
        self.last_pan_steps  = None
        self.last_tilt_steps = None

        self.waiting = False # esperando respuesta a 'pp' o 'tp
        self.query_timer = None # timer de consulta periódica
        self.query_started = None # timestamp de inicio
        self.last_cmd_send_time = None # ultimo comando enviado

        self.query_period = 1 # s entre consultas 'pp'
        self.command_resend_period = 2 # reenvío de setpoint mientras consulto
        self.query_timeout_s = 60.0 # timeout total para confirmar que se llego a la posicion objetivo
        self.tolerance_steps = 0 # tolerancia de coincidencia en pasos

        # feedback
        self.pan_re = re.compile(r'Current\s+Pan\s+position\s+is\s+(-?\d+)', re.IGNORECASE)
        self.tilt_re = re.compile(r'Current\s+Tilt\s+position\s+is\s+(-?\d+)', re.IGNORECASE)

        self.server = ActionServer(
            self,
            PtuSweep,
            'ptu_sweep',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb
            #cancel_callback=self.cancel_cb
        )

    def goal_cb(self, goal_request):
        """ActionServer: aceptación/cancelación"""
        # Rechaza si ya hay una goal activa (este server es mono-objetivo)
        if self.active_goal is not None:
            self.get_logger().warn("PTU ocupado: rechazando nueva meta.")
            return GoalResponse.REJECT # se ignora
        return GoalResponse.ACCEPT # se acepta

    #def cancel_cb(self, goal_handle):
    #    self.get_logger().info("Solicitud de cancelación recibida.")
    #    #self.cancel_evt.set() # avisa que hay cancelación
    #    #self.stop_query_loop() # detiene timers
    #    #self.finish(False, "Cancelado por el usuario")
    #    return CancelResponse.ACCEPT
    
    async def execute_cb(self, goal_handle):
        self.active_goal = goal_handle
        self.done_evt.clear()
        #self.cancel_evt.clear()
        self.success = False
        self.final_message = "OK"

        pan_deg = goal_handle.request.pan_deg
        tilt_deg = goal_handle.request.tilt_deg

        self.target_pan_steps  = grados_a_pasos(pan_deg)
        self.target_tilt_steps = grados_a_pasos(tilt_deg)

        # manda movimiento
        self.publish_feedback(pan_deg, tilt_deg, f"Moviendo a pan={pan_deg}°, tilt={tilt_deg}°")
        self.send_cmd(f"pp{self.target_pan_steps}")
        self.send_cmd(f"tp{self.target_tilt_steps}")
        self.last_cmd_send_time = time.monotonic()

        # Inicia verificación
        self.waiting = True
        self.last_pan_steps  = None
        self.last_tilt_steps = None
        self.query_started = time.monotonic()
        if self.query_timer is None:
            self.query_timer = self.create_timer(self.query_period, self.query_tick)

        # Primera consulta
        self.send_cmd('pp')
        self.send_cmd('tp')

        # Espera a que termine (éxito o timeout/abort)
        self.done_evt.wait()
        #while not self.done_evt.is_set():
        #    if self.cancel_evt.is_set():
        #        goal_handle.canceled()
        #        self.active_goal = None
        #        result = PtuSweep.Result()
        #        result.success = False
        #        result.message = "Cancelado"
        #        return result
        #    time.sleep(0.05)

        # resultado
        result = PtuSweep.Result()
        result.success = self.success
        result.message = self.final_message

        if self.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.active_goal = None
        return result
    
    def query_tick(self):
        #if self.cancel_evt.is_set():
        #    return
    
        if not self.waiting: # si no hay ningun proceso en espera
            return
        
        # ¿llegó? -> se cumple el objetivo de tolerancia
        pan_ok  = (self.last_pan_steps  is not None and
                   abs(self.last_pan_steps  - self.target_pan_steps)  <= self.tolerance_steps)
        tilt_ok = (self.last_tilt_steps is not None and
                   abs(self.last_tilt_steps - self.target_tilt_steps) <= self.tolerance_steps)
        
        if pan_ok and tilt_ok:
            pan_deg  = pasos_a_grados(self.last_pan_steps)
            tilt_deg = pasos_a_grados(self.last_tilt_steps)
            self.publish_feedback(pan_deg, tilt_deg, f"Llegó a pan={pan_deg}°, tilt={tilt_deg}°")
            self.stop_query_loop() # ciclo de pregunta
            self.finish(True, "OK")
            return

        # timeout
        elapsed = time.monotonic() - self.query_started # tiempo transcurrido
        # si se cumple el timeout hay una retroalimentacion
        if elapsed > self.query_timeout_s:
            aprox_pan = pasos_a_grados(self.last_pan_steps) if self.last_pan_steps is not None else 0
            aprox_tilt = pasos_a_grados(self.last_tilt_steps) if self.last_tilt_steps is not None else 0
            self.publish_feedback(aprox_pan, aprox_tilt,
                                   f"Timeout verificando llegada a pan={self.target_pan_steps}, tilt={self.target_tilt_steps}")
            self.stop_query_loop()
            self.finish(False, "Timeout")
            return

        # re-consulta y reinyecta setpoint
        self.send_cmd('pp')
        self.send_cmd('tp')
        now = time.monotonic()
        if self.last_cmd_send_time is not None and (now - self.last_cmd_send_time) >= self.command_resend_period:
            self.send_cmd(f'pp{self.target_pan_steps}')
            self.send_cmd(f'tp{self.target_tilt_steps}')
            self.last_cmd_send_time = now

    def stop_query_loop(self):
        self.waiting = False
        if self.query_timer is not None:
            self.query_timer.cancel()
            self.query_timer = None
        self.last_cmd_send_time = None
    
    def publish_feedback(self, pan_deg: int, tilt_deg: int, status: str):
        if self.active_goal:
            fb = PtuSweep.Feedback()
            fb.current_pan_deg  = int(pan_deg)
            fb.current_tilt_deg = int(tilt_deg)
            fb.status = status
            self.active_goal.publish_feedback(fb)
    
    # publicar comando
    def send_cmd(self, text: str):
        msg = String()
        msg.data = text
        self.cmd_pub.publish(msg)

    def finish(self, ok: bool, msg: str):
        self.success = ok
        self.final_message = msg
        self.done_evt.set()

    def rx_cb(self, msg: String):
        line = msg.data.strip()
        # busca entero con signo en la línea
        m_pan  = self.pan_re.search(line)
        m_tilt = self.tilt_re.search(line)
        if m_pan:
            self.last_pan_steps = int(m_pan.group(1)) # resultado del match
        if m_tilt:
            self.last_tilt_steps = int(m_tilt.group(1))

def main(args=None):
    rclpy.init(args=args)
    node = PtuRoutineNode()
    try:
        executor = MultiThreadedExecutor(num_threads=2) # <- al menos 2 hilos
        executor.add_node(node)
        executor.spin()
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
