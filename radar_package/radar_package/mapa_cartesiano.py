#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarCartesian

class CartPlot(Node):
    def __init__(self):
        super().__init__('cart_plot')
        self.create_subscription(RadarCartesian, 'radar_cartesian_accum', self.cb, 10)

        self.x = np.array([], dtype=np.float32)
        self.y = np.array([], dtype=np.float32)
        self.active_pose_id = -1
        self.last_pose_id = None
        self._need_hard_clear = False
        self.have_data = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        (self.ln,) = self.ax.plot([], [], '.', markersize=1)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Puntos detectados (Cartesianos)')
        self.ax.grid(True, linestyle='--', linewidth=0.5)

        # Etiqueta fija para mostrar pose activo y # de puntos
        self.info_text = self.ax.text(
            0.02, 0.98, 'pose: -  | N: 0',
            transform=self.ax.transAxes, va='top', ha='left',
            fontsize=9, bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='0.7', alpha=0.8)
        )

        plt.show(block=False)
        self.timer = self.create_timer(0.05, self.refresh)

    def cb(self, msg: RadarCartesian):
        pose_id = getattr(msg, 'bunker_pose_id', -1)

        # 2) limpiar buffers si cambia la pose (evita solape entre poses)
        if self.last_pose_id is None or pose_id != self.last_pose_id:
            self.last_pose_id = pose_id
            self._need_hard_clear = True 

        x = np.asarray(msg.x, dtype=np.float32)
        y = np.asarray(msg.y, dtype=np.float32)
        mask = np.isfinite(x) & np.isfinite(y)
        x, y = x[mask], y[mask]

        self.x, self.y = x, y
        self.active_pose_id = pose_id
        self.have_data = (x.size > 0)

    def refresh(self):
        if not self.have_data:
            return
        
        if self._need_hard_clear:
            self.ax.cla()  # limpia TODO el axes
            # recrear artistas y estilos
            (self.ln,) = self.ax.plot([], [], '.', markersize=1)
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.grid(True, linestyle='--', linewidth=0.5)
            self.info_text = self.ax.text(
                0.02, 0.98, '', transform=self.ax.transAxes,
                va='top', ha='left', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='0.7', alpha=0.8)
            )
            self._need_hard_clear = False

        self.ln.set_data(self.x, self.y)

        # Aquí se respetan los rangos reales de los datos, sin forzar cuadrado
        self.ax.relim()
        self.ax.autoscale_view()

        self.ax.set_title(f'Puntos detectados (Cartesianos) — pose activo: {self.active_pose_id}')
        self.info_text.set_text(f'pose: {self.active_pose_id}  |  N: {self.x.size}')

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
