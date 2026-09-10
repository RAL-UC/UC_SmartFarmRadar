#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarCartesian

class CartPlotFlat(Node):
    """
    Se suscribe a /radar_cartesian y grafica en 2D (X,Y) aplanando el mapa (z -> 0).
    Opciones:
      - persist [bool]: acumular puntos (default True)
      - max_points [int]: límite de puntos si persist=True (0 = sin límite)
      - refresh_hz [float]: frecuencia de refresco del plot
    """

    def __init__(self):
        super().__init__('cart_plot_flat')

        # Parámetros
        self.declare_parameter('persist', True)
        self.declare_parameter('max_points', 1000000)
        self.declare_parameter('refresh_hz', 20.0)

        self.persist = bool(self.get_parameter('persist').value)
        self.max_points = int(self.get_parameter('max_points').value)
        self.refresh_hz = float(self.get_parameter('refresh_hz').value)

        # Suscripción
        self.create_subscription(RadarCartesian, 'radar_cartesian', self.cb, 10)

        # Buffers
        self.x = np.array([], dtype=np.float32)
        self.y = np.array([], dtype=np.float32)
        self.have_data = False

        # Figura
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6), num="Radar XY (aplanado z=0)")
        (self.ln,) = self.ax.plot([], [], '.', markersize=1)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Puntos detectados (Cartesianos, z aplanado)')
        self.ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
        self.ax.set_aspect('equal', adjustable='box')
        # Marca del origen
        self.ax.plot(0, 0, '+', markersize=6)

        plt.show(block=False)

        period = 1.0 / max(self.refresh_hz, 1e-3)
        self.timer = self.create_timer(period, self.refresh)

        self.get_logger().info(
            f"cart_plot_flat listo | persist={self.persist}, "
            f"max_points={self.max_points}, refresh_hz={self.refresh_hz}"
        )

    def cb(self, msg: RadarCartesian):
        # Tomamos x,y; ignoramos z (aplanado). Si quisieras “forzar” z=0, sería solo a efectos de publicar otro mensaje,
        # pero para graficar 2D no es necesario.
        x = np.asarray(msg.x, dtype=np.float32)
        y = np.asarray(msg.y, dtype=np.float32)

        if x.size == 0 or y.size == 0:
            return

        mask = np.isfinite(x) & np.isfinite(y)
        x, y = x[mask], y[mask]
        if x.size == 0:
            return

        if self.persist:
            if self.x.size == 0:
                self.x, self.y = x, y
            else:
                self.x = np.concatenate([self.x, x])
                self.y = np.concatenate([self.y, y])

            if self.max_points > 0 and self.x.size > self.max_points:
                excess = self.x.size - self.max_points
                self.x = self.x[excess:]
                self.y = self.y[excess:]
        else:
            # Solo el último mensaje
            self.x, self.y = x, y

        self.have_data = True

    def refresh(self):
        if not self.have_data:
            return

        self.ln.set_data(self.x, self.y)

        # Autoscale con margen
        xmin, xmax = np.min(self.x), np.max(self.x)
        ymin, ymax = np.min(self.y), np.max(self.y)
        dx = (xmax - xmin) if xmax > xmin else 1.0
        dy = (ymax - ymin) if ymax > ymin else 1.0
        pad_x = 0.05 * dx
        pad_y = 0.05 * dy
        self.ax.set_xlim(xmin - pad_x, xmax + pad_x)
        self.ax.set_ylim(ymin - pad_y, ymax + pad_y)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = CartPlotFlat()
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
