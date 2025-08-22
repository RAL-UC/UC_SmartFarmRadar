#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from radar_msg.msg import RadarCartesian

class CartPlot(Node):
    def __init__(self):
        super().__init__('cart_plot')
        self.create_subscription(RadarCartesian, 'radar_cartesian', self.cb, 10)

        self.x = np.array([], dtype=np.float32)
        self.y = np.array([], dtype=np.float32)
        self.have_data = False

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        (self.ln,) = self.ax.plot([], [], '.', markersize=1)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_title('Puntos detectados (Cartesianos)')
        self.ax.grid(True, linestyle='--', linewidth=0.5)

        plt.show(block=False)
        self.timer = self.create_timer(0.05, self.refresh)

    def cb(self, msg: RadarCartesian):
        x = np.asarray(msg.x, dtype=np.float32)
        y = np.asarray(msg.y, dtype=np.float32)
        mask = np.isfinite(x) & np.isfinite(y)
        x, y = x[mask], y[mask]
        if x.size == 0:
            return
        self.x, self.y = x, y
        self.have_data = True

    def refresh(self):
        if not self.have_data:
            return

        self.ln.set_data(self.x, self.y)

        # Aquí se respetan los rangos reales de los datos, sin forzar cuadrado
        self.ax.relim()
        self.ax.autoscale_view()

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
