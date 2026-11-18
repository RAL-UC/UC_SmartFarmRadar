#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
#from typing import Tuple # indicar que una variable, argumento o valor de retorno es una tupla con tipos definidos
#import math
#import time

from std_msgs.msg import Header
#from sensor_msgs.msg import PointCloud2, PointField # nube de puntos 3D y su estructura
#from sensor_msgs_py import point_cloud2 as pc2 # funciones auxiliares en Python para crear y leer PointCloud2 fácilmente

from radar_msg.msg import RadarCartesian

#from std_srvs.srv import Empty

import tf_transformations # operaciones matemáticas con transformaciones 3D, especialmente rotaciones y orientaciones
import tf2_ros # transformacion de coordenadas 
from geometry_msgs.msg import TransformStamped # definicion de mensaje geométricos: posiciones, vectores, poses, transformaciones, etc
from builtin_interfaces.msg import Time as TimeMsg # control de tiempo
from collections import deque # colas de doble extremo
from geometry_msgs.msg import PoseStamped # pose (posición + orientación) asociada a un instante de tiempo y un marco de referencia

class RadarMapAccumulator(Node):
    """
    Acumula puntos de RadarCartesian en un frame fijo (map/odom),
    usando TF2 para transformar cada frame. Aplica voxel grid y/o
    ventana temporal para evitar crecimiento sin límite.
    Publica RadarCartesian y PointCloud2 acumulados.
    Servicio ~/clear_map para limpiar.
    """

    def __init__(self):
        super().__init__('radar_map_accumulator')

        # ---------------- Parámetros ----------------
        self.declare_parameter('fixed_frame', 'radar_sensor') # frame destino para acumular
        #self.declare_parameter('history_secs', 30.0) # ventana temporal (seg) para conservar puntos (0 = infinito)
        #self.declare_parameter('voxel_leaf', 0.10) # tamaño de voxel (m); 0 o <0 desactiva
        #self.declare_parameter('max_points', 300000) # límite duro de puntos
        self.declare_parameter('publish_rate_hz', 5.0) # frecuencia de publicación del acumulado
        self.declare_parameter('gps_max_dt', 10.0)

        p = self.get_parameter
        self.fixed_frame = p('fixed_frame').value
        #self.history_secs = float(p('history_secs').value)
        #self.voxel_leaf = float(p('voxel_leaf').value)
        #self.max_points = int(p('max_points').value)
        self.pub_rate_hz  = float(p('publish_rate_hz').value)
        self.gps_max_dt = float(p('gps_max_dt').value)

        self.fix_buffer = deque(maxlen=2000)

        # --------------- Subs / Pubs ----------------
        self.sub = self.create_subscription(RadarCartesian, 'radar_cartesian', self.cb_points, 10)
        self.pub_rc = self.create_publisher(RadarCartesian, 'radar_cartesian_accum', 10)
        #self.pub_pc = self.create_publisher(PointCloud2, 'radar_cloud_accum', 10)
        self.sub_fix_utm = self.create_subscription(PoseStamped, 'fix_utm', self.cb_fix_utm, 10)   

        # --------------- TF2 (buffer + listener) ---------------
        # se crea el buffer de transformaciones, mantiene hasta x segundos del historial de transformaciones
        self.tf_buffer   = tf2_ros.Buffer(cache_time=rclpy.time.Duration(seconds=3*60.0))
        # suscribirse al tópico /tf y /tf_static automáticamente y va llenando el buffer
        # lanza un hilo separado que procesa los mensajes de TF, así el buffer se va actualizando en paralelo aunque tu nodo esté ocupado en otra cosa
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # --------------- Almacenamiento por robot_pose_id ---------------
        # pose_maps: { pose_id: {"pts": float32[N,3], "ts": float64[N]} }
        self.pose_maps = {}
        # pose activo (último visto). Hasta recibir algo, None.
        self.active_pose_id = None

        # --------------- Almacenamiento ---------------
        # guardamos puntos como float32 y timestamps en float64 (epoch)
        #self._pts = np.empty((0, 3), dtype=np.float32)
        #self._ts  = np.empty((0,),   dtype=np.float64)

        # --------------- Timer de publicación ---------------
        period = 1.0 / max(1e-3, self.pub_rate_hz)
        self.timer = self.create_timer(period, self.publish_accumulated)

        # --------------- Servicios ---------------
        #self.srv_clear_current = self.create_service(Empty, 'clear_map', self.srv_clear_current_cb)
        #self.srv_clear_all     = self.create_service(Empty, 'clear_all_maps', self.srv_clear_all_cb)

        #self.get_logger().info(
        #    f"RadarMapAccumulator escuchando {self.input_topic} → frame fijo '{self.fixed_frame}', "
        #    f"ventana={self.history_secs}s, voxel={self.voxel_leaf} m, max_points={self.max_points}"
        #)

    # ---------- Utilidad: TransformStamped → matriz 4x4 ----------
    @staticmethod
    def transform_to_mat44(tf: TransformStamped) -> np.ndarray:
        # recibe un objeto TransformStamped que contiene:
        # una traslación en x, y, z
        # una rotacion en cuaterniones x, y, z, w 
        t = tf.transform.translation # traslacion
        q = tf.transform.rotation # rotacion en quaterniones
        # construye un array con el cuaternión
        # parte vectorial q.x, q.y, q.z -> direccion del eje de rotacion
        # parte escalar q.w -> relacionado con el angulo de rotacion theta
        # se convierte el quaternion en una matriz de rotacion de 4x4
        # q = (x, y, z, w) = w + xi + yj + zk
        # [[ r11 r12 r13 0 ]
        #  [ r21 r22 r23 0 ]
        #  [ r31 r32 r33 0 ]
        #  [  0   0   0  1 ]]
        # w = cos(theta/2) | (x, y, z) = axis * sin(theta/2)
        # matriz homogenea: matriz 4x4 en 3D representar rotaciones y traslaciones al mismo tiempo
        T = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        # se añade el vector de traslacion a la matriz
        T[0, 3] = t.x
        T[1, 3] = t.y
        T[2, 3] = t.z
        return T
    
    @staticmethod
    def _time_to_sec(stamp) -> float:
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)
    
    def _ensure_pose_slot(self, pose_id: int):
        if pose_id not in self.pose_maps:
            self.pose_maps[pose_id] = {
                "pts": np.empty((0, 3), dtype=np.float32),
                "ts":  np.empty((0,),   dtype=np.float64),
                # --- GPS por punto ---
                "gps_e":   np.empty((0,),   dtype=np.float32),
                "gps_n":   np.empty((0,),   dtype=np.float32),
                "gps_alt": np.empty((0,),   dtype=np.float32),
                "gps_qx":   np.empty((0,),   dtype=np.float32),
                "gps_qy":   np.empty((0,),   dtype=np.float32),
                "gps_qz":   np.empty((0,),   dtype=np.float32),
                "gps_qw":   np.empty((0,),   dtype=np.float32),
                "gps_ts":  np.empty((0,),   dtype=np.float64),
                "gps_frame": "utm_19H",   # se actualiza con la fix recibida
            }

    def _push_fix(self, stamp, e, n, alt, qx, qy, qz, qw, frame_id: str):
        self.fix_buffer.append({    
            "ts": self._time_to_sec(stamp),
            "e": float(e), "n": float(n), "alt": float(alt),
            "qx": float(qx), "qy": float(qy), "qz": float(qz), "qw": float(qw),
            "frame": frame_id or "",
        })

    def _closest_fix(self, ts: float, max_dt: float):
        if not self.fix_buffer:
            return None
        arr_ts = np.fromiter((f["ts"] for f in self.fix_buffer),
                            dtype=np.float64, count=len(self.fix_buffer))
        idx = int(np.argmin(np.abs(arr_ts - ts)))
        best = self.fix_buffer[idx]
        tiempo = np.abs(best["ts"] - ts)
        return best if tiempo <= max_dt else None
    
    def cb_fix_utm(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self._push_fix(msg.header.stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w, msg.header.frame_id)


    # ---------- Callback de puntos entrantes ----------
    def cb_points(self, msg: RadarCartesian):
        # Validación mínima
        if len(msg.x) == 0:
            return
        
        pose_id = int(msg.robot_pose_id)

        if self.active_pose_id is None or pose_id != self.active_pose_id:
            prev = self.active_pose_id
            self.active_pose_id = pose_id
            self._ensure_pose_slot(pose_id)
            self.get_logger().info(
                f"Cambio de robot_pose_id {prev} → {pose_id}. "
                f"Acumulando y publicando ahora el sub-mapa de pose {pose_id}."
            )

        # puntos de entrada
        x = np.asarray(msg.x, dtype=np.float32)
        y = np.asarray(msg.y, dtype=np.float32)
        z = np.asarray(msg.z, dtype=np.float32)
        pts = np.stack([x, y, z], axis=1) # shape (N,3)

        # Obtener transform msg.frame al fixed_frame al tiempo del mensaje
        src_frame = msg.header.frame_id
        stamp = rclpy.time.Time(seconds=msg.header.stamp.sec, nanoseconds=msg.header.stamp.nanosec)

        if src_frame == self.fixed_frame:
            pts_tf = pts.astype(np.float32)
        # tf cada 10hz
        # pide a TF2 la transformación entre dos frames de referencia en un momento concreto
        # frame destino map odom -> después de este paso, todos los puntos quedarán expresados en este frame
        # desde el frame de origen radar_sensor, en verdad es desde el ptu
        # Timeout: cuánto esperar a que el transform aparezca en el buffer de TF2
        # devuelve un TransformStamped
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.fixed_frame,     # target
                    src_frame,            # source
                    stamp,                # at time
                    rclpy.duration.Duration(seconds=0.2) # timeout
                )
            except Exception as e:
                # Si no hay TF puntual, intenta el último disponible (0)
                self.get_logger().warn(f"No TF {src_frame}->{self.fixed_frame} @ {stamp.nanoseconds}ns: {e!r}. Intento latest...")
                try:
                    tf = self.tf_buffer.lookup_transform(self.fixed_frame, src_frame, rclpy.time.Time())
                except Exception as e2:
                    self.get_logger().error(f"Sin TF disponible {src_frame}->{self.fixed_frame}: {e2!r}")
                    return

            # aplicar transform
            T = self.transform_to_mat44(tf)
            # a los puntos de entrada se les concatena una columna de 1s
            # eso permite usar la matriz homogénea T para transformar rotación + traslación en una sola operación.
            pts_h = np.concatenate([pts, np.ones((pts.shape[0], 1), dtype=np.float32)], axis=1) # Nx4
            # se multiplica la matriz de rotacion por todos los puntos homogeneos resultando tambien homogeneo
            # se hace la traspuesta para obtener nuevamente (N, 4)
            # se obtienen las 3 primeras columnas y asegura el tipo de dato
            pts_tf = (T @ pts_h.T).T[:, :3].astype(np.float32)

        # guardar puntos y timestamp
        stamp_sec = self._time_to_sec(msg.header.stamp)
        #stamp_sec = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        slot = self.pose_maps[pose_id]
        fix = self._closest_fix(stamp_sec, self.gps_max_dt)
        if fix is None:
            gps_e   = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_n   = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_alt = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_qx  = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_qy  = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_qz  = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_qw  = np.full(pts_tf.shape[0], np.nan, dtype=np.float32)
            gps_ts  = np.full(pts_tf.shape[0], np.nan, dtype=np.float64)
        else:
            gps_e   = np.full(pts_tf.shape[0], fix["e"],   dtype=np.float32)
            gps_n   = np.full(pts_tf.shape[0], fix["n"],   dtype=np.float32)
            gps_alt = np.full(pts_tf.shape[0], fix["alt"], dtype=np.float32)
            gps_qx  = np.full(pts_tf.shape[0], fix["qx"],  dtype=np.float32)
            gps_qy  = np.full(pts_tf.shape[0], fix["qy"],  dtype=np.float32)
            gps_qz  = np.full(pts_tf.shape[0], fix["qz"],  dtype=np.float32)
            gps_qw  = np.full(pts_tf.shape[0], fix["qw"],  dtype=np.float32)
            gps_ts  = np.full(pts_tf.shape[0], fix["ts"],  dtype=np.float64)
            slot["gps_frame"] = fix["frame"] or slot.get("gps_frame", "")



        slot["pts"] = np.vstack([slot["pts"], pts_tf]) # apilacion de puntos
        # guarda el timestamp de captura de los datos del sensor
        # np.full crea un vector del mismo tamaño que la cantidad de puntos, todos con el mismo timestamp
        slot["ts"]  = np.concatenate([slot["ts"], np.full(pts_tf.shape[0], stamp_sec, dtype=np.float64)])
        slot["gps_e"]    = np.concatenate([slot["gps_e"],   gps_e])
        slot["gps_n"]    = np.concatenate([slot["gps_n"],   gps_n])
        slot["gps_alt"]  = np.concatenate([slot["gps_alt"], gps_alt])
        slot["gps_qx"]   = np.concatenate([slot["gps_qx"],  gps_qx])
        slot["gps_qy"]   = np.concatenate([slot["gps_qy"],  gps_qy])
        slot["gps_qz"]   = np.concatenate([slot["gps_qz"],  gps_qz])
        slot["gps_qw"]   = np.concatenate([slot["gps_qw"],  gps_qw])
        slot["gps_ts"]   = np.concatenate([slot["gps_ts"],  gps_ts])
        

    # ---------- Compactación: ventana de tiempo, tope de puntos, voxel ----------
    #def compact(self):
    #    # 1) Ventana temporal
    #    if self.history_secs > 0.0:
    #        cutoff = time.time() - self.history_secs
    #        mask = self._ts >= cutoff
    #        self._pts = self._pts[mask]
    #        self._ts  = self._ts[mask]
    #
    #    # 2) Límite duro de puntos (mantén los más recientes)
    #    if self._pts.shape[0] > self.max_points:
    #        keep = self._pts.shape[0] - self.max_points
    #        # descarta los más antiguos
    #        self._pts = self._pts[-self.max_points:, :]
    #        self._ts  = self._ts[-self.max_points:]
    #
    #    # 3) Voxel grid (downsample espacial)
    #    if self.voxel_leaf is not None and self.voxel_leaf > 0.0 and self._pts.shape[0] > 0:
    #        leaf = self.voxel_leaf
    #        keys = np.floor(self._pts / leaf).astype(np.int64)
    #        # usa vista estructurada para agrupar por llave (x,y,z discretas)
    #        keys_view = keys.view([('x', np.int64), ('y', np.int64), ('z', np.int64)]).reshape(-1)
    #        # ordena por llave
    #        order = np.argsort(keys_view, kind='mergesort')
    #        keys_sorted = keys_view[order]
    #        pts_sorted  = self._pts[order]
    #        ts_sorted   = self._ts[order]
    #
    #        # agrupa (promedio por voxel)
    #        uniq, first_idx = np.unique(keys_sorted, return_index=True)
    #        # índices de fin de grupo
    #        last_idx = np.concatenate([first_idx[1:] - 1, np.array([pts_sorted.shape[0]-1])])
    #
    #        new_pts = []
    #        new_ts  = []
    #        for a, b in zip(first_idx, last_idx):
    #            # promedio de puntos del voxel (podrías usar el último timestamp o promedio)
    #            new_pts.append(np.mean(pts_sorted[a:b+1], axis=0))
    #            new_ts.append(np.max(ts_sorted[a:b+1]))  # conservamos el ts más reciente del voxel
    #
    #        self._pts = np.vstack(new_pts).astype(np.float32)
    #        self._ts  = np.array(new_ts, dtype=np.float64)

    # ---------- Publicación periódica del acumulado ----------
    def publish_accumulated(self):
        pose_id = self.active_pose_id
        if pose_id is None:
            return
        
        slot = self.pose_maps.get(pose_id, None)

        if slot is None or slot["pts"].shape[0] == 0:
            return

        # Header en el frame fijo con tiempo actual
        hdr = Header() # crea un header 
        hdr.stamp = self.get_clock().now().to_msg() # tiempo actual del reloj del nodo
        hdr.frame_id = self.fixed_frame # frame fijo al que se transforman todos los puntos

        # publicar RadarCartesian acumulado
        msg_rc = RadarCartesian()
        msg_rc.header = hdr
        pts = slot["pts"]
        msg_rc.x = pts[:, 0].tolist()
        msg_rc.y = pts[:, 1].tolist()
        msg_rc.z = pts[:, 2].tolist()

        msg_rc.gps_e    = slot["gps_e"].astype(np.float32).tolist()
        msg_rc.gps_n    = slot["gps_n"].astype(np.float32).tolist()
        msg_rc.gps_alt  = slot["gps_alt"].astype(np.float32).tolist()
        msg_rc.gps_qx   = slot["gps_qx"].astype(np.float32).tolist()
        msg_rc.gps_qy   = slot["gps_qy"].astype(np.float32).tolist()
        msg_rc.gps_qz   = slot["gps_qz"].astype(np.float32).tolist()
        msg_rc.gps_qw   = slot["gps_qw"].astype(np.float32).tolist()
        msg_rc.gps_frame = slot.get("gps_frame", "")

        msg_rc.robot_pose_id = int(pose_id)

         # construir lista de Time por punto
        stamps = []
        for ts in slot["ts"]: 
            t = TimeMsg()
            t.sec = int(ts)
            t.nanosec = int((ts - int(ts)) * 1e9)
            stamps.append(t)
        msg_rc.stamps = stamps
        self.pub_rc.publish(msg_rc)

        # 2) PointCloud2 acumulado (para RViz)
        #fields = [
        #    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        #    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        #    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        #]
        #pc2_msg = pc2.create_cloud(hdr, fields, self._pts)
        #self.pub_pc.publish(pc2_msg)

    # ---------- Servicio para limpiar ----------
    #def srv_clear_current_cb(self, req, res):
    #    if self.active_pose_id is None or self.active_pose_id not in self.pose_maps:
    #        self.get_logger().info("No hay sub-mapa activo para limpiar.")
    #        return res
    #    pose_id = self.active_pose_id
    #    self.pose_maps[pose_id]["pts"] = np.empty((0, 3), dtype=np.float32)
    #    self.pose_maps[pose_id]["ts"]  = np.empty((0,),   dtype=np.float64)
    #    self.get_logger().info(f"Sub-mapa del robot_pose_id {pose_id} limpiado.")
    #    return res
    #
    #def srv_clear_all_cb(self, req, res):
    #    self.pose_maps.clear()
    #    self.active_pose_id = None
    #    self.get_logger().info("Todos los sub-mapas (por robot_pose_id) fueron limpiados.")
    #    return res


def main(args=None):
    rclpy.init(args=args)
    node = RadarMapAccumulator()
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
