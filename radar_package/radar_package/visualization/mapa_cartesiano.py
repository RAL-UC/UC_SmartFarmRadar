#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from radar_msg.msg import RadarCartesian
from builtin_interfaces.msg import Time as TimeMsg

# direccion de guardado de datos
SAVE_DIR = Path('/home/dammr/Desktop/magister_ws/UC_SmartFarmRadar/datos/experimetos_pao')

class CartPlot(Node):
    def __init__(self):
        super().__init__('cart_plot')
        # suscriptor
        self.create_subscription(RadarCartesian, 'radar_cartesian_accum', self.cb, 10)

        self.last_pose_id = None # id posiciob bunker
        self.last_pts = None # (N,3) float32 coordenadas
        self.last_ts = None # (N,) float64 tiempo

        # coordenadas gps
        self.last_gps_e = None
        self.last_gps_n = None
        self.last_gps_alt = None
        self.last_gps_qx = None
        self.last_gps_qy = None
        self.last_gps_qz = None
        self.last_gps_qw = None
        self.last_gps_ts = None
        self.last_gps_frame = ""

        # configuracion de visualizacion
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        (self.ln,) = self.ax.plot([], [], '.', markersize=5, color='green') # puntos verdes
        self.map_size = 10.0 # lado del cuadro fijo [m]
        self.center_x = 0.0 # centro X [m]
        self.center_y = 0.0 # centro Y [m]
        self.apply_fixed_view() # encuadre inicial
        self.ax.set_xlabel('X [m]') # detalle eje
        self.ax.set_ylabel('Y [m]') # detalle eje
        self.ax.set_title('Puntos detectados (Cartesianos)')
        self.ax.grid(True, linestyle='--', linewidth=0.5)

        # Etiqueta fija para mostrar pose activo y cantidad de puntos
        #self.info_text = self.ax.text(
        #    0.02, 0.98, 'pose: - | N: 0',
        #    transform=self.ax.transAxes, va='top', ha='left',
        #    fontsize=9, bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='0.7', alpha=0.8)
        #)

        # data
        SAVE_DIR.mkdir(parents=True, exist_ok=True)
        self.timer = self.create_timer(0.05, self.refresh) # timer de refresco de visualizacion

        self.cur_x = np.array([], dtype=np.float32)
        self.cur_y = np.array([], dtype=np.float32)
        self.active_pose_id = -1
        self.need_hard_clear = False
        self.have_data = False

        plt.show(block=False)

    @staticmethod
    def time_to_sec(t: TimeMsg) -> float:
        """transportar de nano segundos a segundos"""
        return float(t.sec) + float(t.nanosec) * 1e-9
    
    def apply_fixed_view(self):
        """configuracion de vista fija"""
        half = self.map_size / 2.0
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.set_xlim(self.center_x - half, self.center_x + half)
        self.ax.set_ylim(self.center_y - half, self.center_y + half)
    
    def save_snapshot(self, pose_id: int, pts: np.ndarray, ts: np.ndarray):
        """Guarda el último batch de una pose en .npz (points Nx3, timestamps N)."""
        if pts is None or pts.size == 0 or pose_id is None:
            return
        fname = SAVE_DIR / f'pose_{pose_id}.npz'
        def arr_or_empty(a, dtype): # informacion o vacio
            if a is None:
                return np.empty((0,), dtype=dtype)
            return np.asarray(a, dtype=dtype)
        
        np.savez(
            fname,
            points=pts.astype(np.float32), # Nx3
            timestamps=ts.astype(np.float64), # N
            pose_id=np.int32(pose_id),
            # ----- GPS opcional por punto -----
            gps_e=arr_or_empty(self.last_gps_e, np.float32),
            gps_n=arr_or_empty(self.last_gps_n, np.float32),
            gps_alt=arr_or_empty(self.last_gps_alt, np.float32),
            gps_qx=arr_or_empty(self.last_gps_qx, np.float32),
            gps_qy=arr_or_empty(self.last_gps_qy, np.float32),
            gps_qz=arr_or_empty(self.last_gps_qz, np.float32),
            gps_qw=arr_or_empty(self.last_gps_qw, np.float32),
            gps_ts=arr_or_empty(self.last_gps_ts, np.float64),
            gps_frame=np.array(self.last_gps_frame, dtype='U32')
        )
        self.get_logger().info(f'[CartPlot] Guardado {fname} | N={pts.shape[0]} | pose={pose_id}')

    def cb(self, msg: RadarCartesian):
        pose_id = getattr(msg, 'bunker_pose_id', -1)

        # limpiar buffers si cambia la pose (evita solape entre poses)
        if self.last_pose_id is None or pose_id != self.last_pose_id:
            #self.save_snapshot(self.last_pose_id, self.last_pts, self.last_ts)
            self.need_hard_clear = True 

        # orden de data para guardado
        x_raw = np.asarray(msg.x, dtype=np.float32)
        y_raw = np.asarray(msg.y, dtype=np.float32)
        z_raw = np.asarray(msg.z, dtype=np.float32)
        mask = np.isfinite(x_raw) & np.isfinite(y_raw) & np.isfinite(z_raw)
        x, y, z = x_raw[mask], y_raw[mask], z_raw[mask]
        pts = np.stack([x, y, z], axis=1)

        stamps = getattr(msg, 'stamps', None)
        if stamps and len(stamps) == len(x_raw):
            ts_all = np.array([self.time_to_sec(t) for t in stamps], dtype=np.float64)
            ts = ts_all[mask]
        else:
            t0 = float(msg.header.stamp.sec) + 1e-9 * float(msg.header.stamp.nanosec)
            ts = np.full((pts.shape[0],), t0, dtype=np.float64)

        def align_opt(name, dtype):
            arr = getattr(msg, name, None)
            if arr is None or len(arr) == 0:
                return np.full((pts.shape[0],), np.nan, dtype=dtype)
            arr_np = np.asarray(arr, dtype=dtype)
            if arr_np.size == len(x_raw):
                return arr_np[mask]
            if arr_np.size == pts.shape[0]:
                return arr_np
            # tamaño inesperado -> rellena con NaN manteniendo longitud N
            return np.full((pts.shape[0],), np.nan, dtype=dtype)
        
        gps_e = align_opt('gps_e', np.float32)
        gps_n = align_opt('gps_n', np.float32)
        gps_alt = align_opt('gps_alt', np.float32)
        gps_qx = align_opt('gps_qx', np.float32)
        gps_qy = align_opt('gps_qy', np.float32)
        gps_qz = align_opt('gps_qz', np.float32)
        gps_qw = align_opt('gps_qw', np.float32)

        gps_stamps = getattr(msg, 'gps_stamps', None)
        if gps_stamps and len(gps_stamps) == len(x_raw):
            gps_ts_all = np.array([self.time_to_sec(t) for t in gps_stamps], dtype=np.float64)
            gps_ts = gps_ts_all[mask]
        elif gps_stamps and len(gps_stamps) == pts.shape[0]:
            gps_ts = np.array([self.time_to_sec(t) for t in gps_stamps], dtype=np.float64)
        else:
            # si no viene gps_stamps por punto, replica header o deja NaN según prefieras
            gps_ts = np.full((pts.shape[0],), np.nan, dtype=np.float64)

        gps_frame = getattr(msg, 'gps_frame', "")
        
        self.last_pose_id = pose_id
        self.last_pts = pts
        self.last_ts = ts

        self.last_gps_e = gps_e
        self.last_gps_n = gps_n
        self.last_gps_alt = gps_alt
        self.last_gps_qx = gps_qx
        self.last_gps_qy = gps_qy
        self.last_gps_qz = gps_qz
        self.last_gps_qw = gps_qw
        self.last_gps_ts = gps_ts
        self.last_gps_frame = gps_frame


        self.cur_x = x
        self.cur_y = y
        self.active_pose_id = pose_id
        self.have_data = (pts.shape[0] > 0)

    def refresh(self):
        if not self.have_data:
            return
        
        if self.need_hard_clear:
            self.ax.cla()  # limpia todo el axes
            # recrear figura y estilos
            (self.ln,) = self.ax.plot([], [], '.', markersize=5, color='green')
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.grid(True, linestyle='--', linewidth=0.5)
            #self.info_text = self.ax.text(
            #    0.02, 0.98, '', transform=self.ax.transAxes,
            #    va='top', ha='left', fontsize=9,
            #    bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='0.7', alpha=0.8)
            #)
            self.need_hard_clear = False

        self.ln.set_data(self.cur_x, self.cur_y)

        # Aquí se respetan los rangos reales de los datos, sin forzar cuadrado
        #self.ax.relim()
        #self.ax.autoscale_view()
        self.apply_fixed_view()
        self.ax.set_title(f"XY Map Tree-Facing Capture [{self.active_pose_id}]")
        #self.info_text.set_text(f'pose: {self.active_pose_id}  |  N: {self.cur_x.size}')

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CartPlot()
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
